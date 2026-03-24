import torch
from PIL import Image
from langgraph.graph import StateGraph, START, END
from models import logic_llm, vlm_model, processor
from state import AgentState
from prompts import MISSIONS
from tools import execute_action
import json
from prompts import MISSIONS, ORCHESTRATOR_SYSTEM
# NODE: Visual Perception (SmolVLM)
def perception_node(state: AgentState):
    mission = MISSIONS[state["mission_key"]]
    user_prompt = mission["user"]
    
    if state["mission_key"] == "nav" and state["target_object"]:
        user_prompt = user_prompt.replace("[TARGET]", state["target_object"])

    image = Image.open(state["image_path"])
    
    # SmolVLM processing
    inputs = processor(text=user_prompt, images=image, return_dict=True, return_tensors="pt").to("cuda")
    output_ids = vlm_model.generate(**inputs, max_new_tokens=100)
    description = processor.batch_decode(output_ids, skip_special_tokens=True)[0]
    
    return {"visual_analysis": description}

# NODE: Reasoning & Decision (GPT-OSS 120B)
def cognition_node(state: AgentState):
    mission = MISSIONS[state["mission_key"]]
    
    # We use "high reasoning" effort for mission-critical tasks
    system_msg = mission["system"] + "\nOutput format: CODE | DESCRIPTION"
    
    response = logic_llm.invoke([
        {"role": "system", "content": system_msg},
        {"role": "user", "content": f"VLM Analysis: {state['visual_analysis']}"}
    ])
    parts = response.content.split("|")
    decision = parts[0].strip().upper()
    description = parts[1].strip() if len(parts) > 1 else "Target analyzed."
    
    return {"final_decision": decision, "visual_analysis": description}

# ... (Keep your existing imports and Node functions as they are)

# BUILD THE GRAPH (Keep this outside the class for efficiency)
builder = StateGraph(AgentState)
builder.add_node("vision", perception_node)
builder.add_node("brain", cognition_node)
builder.add_edge(START, "vision")
builder.add_edge("vision", "brain")
builder.add_edge("brain", END)
compiled_workflow = builder.compile()

class DroneAgent:
    def __init__(self, default_mission="guard"):
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
