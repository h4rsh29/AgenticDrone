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
def perception_node(state: AgentState):
    mission = MISSIONS[state["mission_key"]]
    user_prompt = mission["user"]
    
    if state["mission_key"] == "nav" and state["target_object"]:
        user_prompt = user_prompt.replace("[TARGET]", state["target_object"])

    image = Image.open(state["image_path"])
    
    # SmolVLM processing
    inputs = processor(text=user_prompt, images=image, return_tensors="pt").to("cuda")
    # UPDATED: Add repetition_penalty and do_sample to stop the "looping" text
    output_ids = vlm_model.generate(
        **inputs, 
        max_new_tokens=40,      # Smaller limit keeps descriptions concise
        repetition_penalty=1.7, # Stops the AI from saying the same thing twice
        do_sample=False,        
        #temperature=0         # Keeps the AI focused on the image
    )
    
    full_text = processor.batch_decode(output_ids, skip_special_tokens=True)[0]
    description = full_text[len(user_prompt):].strip()

    # --- DEBUG PRINT: Add this line here ---
    print(f"\n[RAW VLM OUTPUT]: {description}\n")
    
    # 1. NEW ROBUST SCRUBBER: Catch code blocks and weird character patterns
    garbage_triggers = [
        "[/path]", "123456789|", "$=>", "hasclass", ".gitignore", "max(min", 
        "x86", "include", "documentclass", "html", "copyright", "if applicable", "nswer:", "question:", "f you see", "answer with", "otherwise answer",
        "appropriate label", "label (e g", "[/path]", "$=>", "hasclass", "matter what you say", "yes or no questions", "nswer:", "question:", 
        "[/path]", "123456789|", "appropriate label", "animated 3d render", "people standing" ,"Answer as if you are an AI", "escribe them", "dentify these",  ". . . .", 
        "Index Value", "sqft", "ft²", "BLEEPING DOG", "Failures in the System","Surface Condition Index", "infinity is not a number", "0% of surface","escribe them", "dentify these"
    ]
    
    # 2. LOOP DETECTOR: Catch 'ntt ntt ntt' or repeated character patterns
    if re.search(r'((\. )|(\.)){4,}', description) or re.search(r'(.)\1{4,}', description):
        description = "Structure analysis: Component is visually intact."

    # 3. BACKGROUND FILTER: Remove mentions of people/cars from the structural report
    # If the AI starts talking about objects instead of concrete, it's distracted
    if "people" in description.lower() or "person" in description.lower() or "vehicle" in description.lower():
        # Keep the text BEFORE it gets distracted, or reset if it's mostly noise
        description = description.split("The image is")[0].strip()
        if len(description) < 15:
            description = "Structure analysis: Component is visually intact."
    # Calculate character density to catch code-like hallucinations
    special_chars = sum(1 for c in description if c in "[]{}$_|=<>")
    char_density = special_chars / len(description) if len(description) > 0 else 0

    fault_keywords = ["crack", "puddle", "moisture", "fault", "damage", "spall", "spalling", "brick", "rust", "pitted", "rough", "broken"]
    is_fault_detected = any(k in description.lower() for k in fault_keywords)
    
    # Robust scrubbing: If it looks like code OR contains a trigger, reset it
    if any(t in description.lower() for t in garbage_triggers) or char_density > 0.05:
        description = "Structure analysis: Component is visually intact."
    elif not is_fault_detected and len(description) > 180:
        description = "Structure analysis: Component is visually intact."

    description = description.replace("\n", " ").strip()
    return {"visual_analysis": description}
    



# NODE: Reasoning & Decision (GPT-OSS 120B)
def cognition_node(state: AgentState):
    mission = MISSIONS[state["mission_key"]]
    
    context = f"YOLO: {state['yolo_report']} | VLM: {state['visual_analysis']}"
    
    response = llm_with_tools.invoke([
        {"role": "system", "content": mission["system"]},
        {"role": "user", "content": context}
    ])
    # 2. ROBUST DECISION PARSING
    raw_content = response.content.split("|")[0].strip()
    # Fallback: If the LLM returns a full sentence instead of a CODE, force it to 'PATH_SAFE'
    decision = raw_content if len(raw_content) < 20 else "PATH_SAFE"
     
    if state["mission_key"] == "inspection":
        # Check the actual analysis for keywords, NOT just the GPT decision
        # This ensures that if VLM says "Jagged spall", it becomes CRITICAL
        # This bypasses the LLM's tendency to be "polite" or vague.
        vlm_text = state['visual_analysis'].upper()
        keywords = ["SPALL", "BRICK", "HOLE", "CRACK", "FAULT", "DAMAGE", "PITTED", "JAGGED", "SURFACE AREA", "FLAW", "DEFECT", "BROKEN"]
        
        if any(x in vlm_text for x in keywords):
            final_decision = "CRITICAL" # Chart will show RED
        else:
            final_decision = "HEALTHY"  # Chart will show GREEN
    else:
        final_decision = decision 
        
    return {"final_decision": final_decision, "messages": [response]}

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
