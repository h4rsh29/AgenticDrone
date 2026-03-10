# Agentic Drone: Autonomous Navigation

An autonomous drone agent built with **ROS 2 Humble** and **PX4**, featuring a Vision-Language-Action (VLA)  for intelligent obstacle avoidance.



## 🚀 System Architecture
- **Perception Layer**: YOLOv8 (Visual) + 3D LiDAR (Depth) Fusion.
- **Decision Layer**: Moondream VLM (Vision-Language Model) running locally via Ollama.
- **Action Layer**: PX4 Offboard Control via MAVROS.

## 🛠️ Key Technical Features
- **Hardware Optimization**: Migrated from larger models to **Moondream** to ensure real-time inference (<200ms) on 4GB VRAM (NVIDIA RTX 3050).
- **Safety Protocol**: Dual-check safety system. The drone triggers an `EMERGENCY STOP` if obstacles are detected within 1.5m during flight.
- **Dynamic Waypoints**: The agent calculates unit vectors toward user-defined goals while adjusting heading (Yaw) based on VLA suggestions.



## 📁 Repository Structure
- `perception_nodes/`: YOLO detection and LiDAR filtering logic.
- `navigation_nodes/`: PX4 Offboard control and waypoint management.
- `models/`: Trained YOLO weights (`yolo26n.pt`).

## ⚙️ Requirements
- ROS 2 Humble
- PX4 Autopilot / Gazebo Harmonic
- Python libs: `ultralytics`, `ollama`, `rclpy`, `opencv-python`
