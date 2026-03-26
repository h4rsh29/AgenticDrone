# state.py
from typing import TypedDict, Optional, List, Dict, Any # Added Dict and Any here

class AgentState(TypedDict):
    mission_key: str           # e.g., "nav", "surveillance"
    image_path: str            # Path to the latest frame
    visual_analysis: str       # VLM's description of the scene
    final_decision: str        # STOP, PATH_SAFE, LEFT, RIGHT
    target_object: Optional[str]
    history: List[str]         # To maintain context over time
    yolo_report: str           # YOLO detection summary
    detection_boxes: List[Dict[str, Any]] 