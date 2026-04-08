"AgenticDrone: A ROS 2 & PX4 Flight Stack for Edge-AI Inspection"


AgenticDrone is a ROS 2-powered autonomous system designed for structural health monitoring and tactical surveillance. It utilizes Vision-Language Models (VLM) and YOLO to perform high-level reasoning on structure integrity, detecting defects like cracks, spalling, and moisture ingress through multi-sensor fusion.

🚀 Key Features

PX4-Powered Flight: Fully integrated with the PX4 Autopilot stack for reliable Offboard control and MAVROS telemetry.

Multi-Mission Autonomy: Seamlessly switches between Structural Inspection, Tactical Surveillance, and Precision Navigation using a LangGraph-based mission state machine.

Sensor Fusion (RGB + Thermal): Integrates standard RGB vision with Thermal (Ironbow/Magma) imaging to detect internal moisture and thermal anomalies in real-time.

VLM-Powered Cognition: Leverages SmolVLM-500M and a Senior Structural Engineer LLM persona to interpret visual data and generate professional technical reports.

Automated Reporting: Generates coordinate-aware, time-stamped logs in inspection_report.txt and surveillance_log.txt, including evidence capture for critical faults.

Edge Optimized: Specifically tuned for 4GB VRAM hardware (like the RTX 3050) using 4-bit quantization and efficient image scaling.

🛠️ System Requirements
Autopilot: PX4 Autopilot (v1.14 or higher)

OS: Ubuntu 22.04 (Jammy Jellyfish)

ROS 2: Humble Hawksbill

Simulation: Gazebo Sim (Garden/Harmonic)

Hardware: Minimum 4GB VRAM (NVIDIA RTX series recommended)

Python Dependencies: torch, transformers, langgraph, bitsandbytes, opencv-python, pillow

📦 Installation & Setup

1. PX4 Autopilot Installation
Ensure you have the PX4 Autopilot source code and dependencies installed in your home directory:

Bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

2. Environment Configuration
Ensure the bitsandbytes CUDA 13.x libraries are in your path for the VLM to run correctly on your RTX 3050:

Bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/harsh/.local/lib/python3.10/site-packages/nvidia/cu13/lib

3. Project Setup
Bash
git clone https://github.com/h4rsh29/AgenticDrone.git
cd AgenticDrone
export PYTHONPATH=$PYTHONPATH:$(pwd)


🎮 Execution Guide
Follow these steps in separate terminals to launch the full autonomous stack:

Terminal 1: PX4 SITL & Gazebo

cd ~/PX4-Autopilot
PX4_GZ_WORLD=default make px4_sitl gz_x500_depth
Bash
# Set parameters for GPS-less arming and offboard control
param set NAV_DLL_ACT 0
param set COM_ARM_WO_GPS 1
param save

Terminal 2: MAVROS & Home Position

Bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

Terminal 3: Navigation Node

Bash
cd drone_ws
source install/setup.bash
ros2 run drone_nav waypoint_node

Terminal 4: Pose Telemetry (Optional)

Bash
ros2 topic echo /mavros/local_position/pose

Terminal 5: AI Brain (VLA Node)

Bash
cd AgenticDrone
python3 decision_nodes/node.py


📁 Project Structure
node.py: The main ROS 2 entry point. Handles image acquisition, thermal processing, and the primary UI thread.

agent.py: Contains the LangGraph definition, YOLO detection node, and VLM perception logic.

prompts.py: The "Mission Library" containing system instructions for the Engineer and Analyst personas.

models.py: Centralized model loading with 4-bit quantization settings.

default.sdf: The Gazebo world containing the Concrete and Steel Bridge models with pre-defined structural defects.

📊 Sample Output
Structural Inspection
Plaintext
[STATUS: CRITICAL] | Visible cracks and surface imperfections indicate structural degradation of the concrete wall at X: -9.52.
EVIDENCE: fault_1775628906_vis.jpg
Tactical Surveillance
Plaintext
[STATUS: ACTIVE] | CROWDED scene with 7 persons centrally located and 4 cars positioned on the left ramp.
IMG: detection_1775636092.jpg
