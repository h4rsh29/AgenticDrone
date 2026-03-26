# tools.py
from langchain_core.tools import tool
import logging

logger = logging.getLogger("DroneTools")

@tool
def pilot_command(action: str, description: str):
    """
    Executes a flight maneuver or mission update.
    Args:
        action: The command (STOP, STEER_LEFT, STEER_RIGHT, PATH_SAFE, LAND)
        description: A brief reason for the action for logging.
    """
    action = action.upper()
    logger.info(f"🛠️ TOOL EXECUTION: {action} | {description}")
    return f"Action {action} confirmed: {description}"

# List of tools available to the LangGraph
drone_tools = [pilot_command]