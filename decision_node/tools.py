# tools.py
from langchain_core.tools import tool
import logging

logger = logging.getLogger("DroneTools")

@tool
def execute_flight_path(target_name: str, x: float, y: float, z: float, on_arrival: str):
    """
    Commands the drone to fly to a specific coordinate.
    Args:
        target_name: Name of the destination (e.g. 'bridge', 'takeoff_pad')
        on_arrival: What to do when reached. Options: 'LAND', 'WAIT_FOR_COMMAND', 'START_INSPECTION'
    """
    return f"FLYING_TO_{target_name.upper()}_THEN_{on_arrival.upper()}"

@tool
def trigger_mission_phase(mission_type: str):
    """
    Immediately switches the drone's behavior mode.
    Args:
        mission_type: 'inspection', 'surveillance', or 'manual_hover'
    """
    return f"SWITCHING_TO_{mission_type.upper()}"
# List of tools available to the LangGraph
drone_tools = [execute_flight_path, trigger_mission_phase]
