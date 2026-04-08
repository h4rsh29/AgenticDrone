
import torch
from PIL import Image
from langgraph.graph import StateGraph, START, END
from models import logic_llm, vlm_model, processor
from state import AgentState
from prompts import MISSIONS
import json
from prompts import MISSIONS, ORCHESTRATOR_SYSTEM
from models import yolo_model
from langgraph.prebuilt import ToolNode
from tools import drone_tools
import re

# Bind tools to the LLM
llm_with_tools = logic_llm.bind_tools(drone_tools)

def detection_node(state: AgentState):
    image = Image.open(state["image_path"])
    # YOLO26 provides NMS-free inference, perfect for 2026 drone surveillance
    results = yolo_model(image, conf=0.45)[0] 
    
    found_labels = []
    boxes_data = [] # New list for the squares
    
    for box in results.boxes:
        label = yolo_model.names[int(box.cls[0])]
        conf = float(box.conf[0])
        # Get coordinates: [x1, y1, x2, y2]
        coords = box.xyxy[0].tolist() 
        found_labels.append(label)
        boxes_data.append({"label": label, "box": coords, "conf": conf})

    return {
        "yolo_report": f"YOLO sees: {', '.join(found_labels)}" if found_labels else "Clear",
        "detection_boxes": boxes_data 
    }


# NODE: Visual Perception (SmolVLM)
# agent.py

def perception_node(state: AgentState):
    mission = MISSIONS[state["mission_key"]]
    user_prompt = mission["user"]
    
    if state["mission_key"] == "nav" and state["target_object"]:
        user_prompt = user_prompt.replace("[TARGET]", state["target_object"])

    image = Image.open(state["image_path"])
    
    # 1. Process with SmolVLM
    inputs = processor(text=user_prompt, images=image, return_tensors="pt").to("cuda")
    output_ids = vlm_model.generate(
        **inputs, 
        max_new_tokens=40,      
        repetition_penalty=1.5, 
        do_sample=False,        
    )
    
    full_text = processor.batch_decode(output_ids, skip_special_tokens=True)[0]
    # Extract only the completion after the prompt
    description = full_text[len(user_prompt):].strip()

    # --- DEBUG PRINT ---
    print(f"\n[RAW VLM OUTPUT]: {description}\n")
    
    # 1. NEW SCRUBBER: Catch 'people' talk and metadata hallucinations
    people_words = ["person", "man", "woman", "t-shirt", "shirt", "walking", "pedestrian"]
    garbage_triggers = ["tbody", "304", "10/24", "tx-", "[/path]", "rationale", "if task"]
    
    # 2. If the VLM only talks about people, force a neutral surface report
    if any(p in description.lower() for p in people_words) and len(description) < 60:
        description = "Surface is currently obstructed by personnel; proceeding with base scan."
    
    # 3. Standard garbage reset
    if any(t in description.lower() for t in garbage_triggers) or len(description) < 3:
        description = "Surface appears uniform and intact."

    return {"visual_analysis": description.replace("\n", " ").strip()}
    
# NODE: Reasoning & Decision (GPT-OSS 120B)
def cognition_node(state: AgentState):
    mission = MISSIONS[state["mission_key"]]
    
    # Context for the Senior Engineer (LLM)
    context = f"YOLO: {state['yolo_report']} | VLM_RAW: {state['visual_analysis']}"
    
    response = llm_with_tools.invoke([
        {"role": "system", "content": mission["system"]},
        {"role": "user", "content": f"Create a technical report from this data: {context}"}
    ])

    # 1. Extract the components from the LLM response
    # Format expected: CODE | Description
    parts = response.content.split("|")
    final_code = parts[0].strip()
    # The LLM now 'fine-tunes' the messy VLM text into the proper report description
    professional_description = parts[-1].strip()
     
    # 4. ROBUST STATUS LOGIC
    vlm_raw = state['visual_analysis'].lower()
    # Manual override: If the VLM actually mentions damage keywords, force CRITICAL
    defect_keywords = ["reddish", "jagged", "pitted", "crack", "stain", "rust", "spall", "hole", "brick","rough"]
    vlm_saw_defect = any(k in vlm_raw for k in defect_keywords)

    if state["mission_key"] == "inspection":
        if vlm_saw_defect or "CRITICAL" in final_code.upper():
            status = "CRITICAL"
        else:
            status = "HEALTHY"
    else:
        status = final_code if len(final_code) < 20 else "PATH_SAFE"

    return {
        "final_decision": status, 
        "visual_analysis": professional_description, 
        "messages": [response]
    }

# BUILD THE GRAPH 
builder = StateGraph(AgentState)
builder.add_node("detect", detection_node) # Fast detections
builder.add_node("vision", perception_node)
builder.add_node("brain", cognition_node)
builder.add_node("tools", ToolNode(drone_tools))


builder.add_edge(START, "detect")
builder.add_edge("detect", "vision")
builder.add_edge("vision", "brain")

def should_continue(state: AgentState):
    if state.get("messages") and state["messages"][-1].tool_calls:
        return "tools"
    return END

builder.add_conditional_edges("brain", should_continue)
builder.add_edge("tools", END)
compiled_workflow = builder.compile()

class DroneAgent:
    def __init__(self, default_mission="nav"):
        """
        Initializes the agent with a default mission from prompts.py.
        """
        self.current_mission = default_mission
        self.target_object = None # For 'nav' missions

    def understand_command(self, user_text):
        """Turns your text into a drone mission plan"""
        response = logic_llm.invoke([
            {"role": "system", "content": ORCHESTRATOR_SYSTEM},
            {"role": "user", "content": user_text}
        ])    
        # Robust JSON parsing
        content = response.content.strip()
        if "```json" in content:
            content = content.split("```json")[1].split("```")[0].strip()
    
        data = json.loads(content)
    
        # FIX: If the AI returns a list, take the first dictionary
        if isinstance(data, list):
            return data[0]
        return data
    def get_decision(self, image_path: str, lidar_dist: float, target="person"):
        self.target_object = target
        """
        The bridge method called by node.py. 
        It handles mission logic before running the AI graph.
        """
        # 1. AUTO-SWITCH: If LiDAR is close, force 'avoidance' mission
        active_mission = self.current_mission
        if lidar_dist < 3.0:
            active_mission = "avoidance"


        # 2. Prepare the Input State
        initial_state = {
            "mission_key": active_mission,
            "image_path": image_path,
            "target_object": self.target_object,
            "history": [] 
        }

        return compiled_workflow.invoke(initial_state)
