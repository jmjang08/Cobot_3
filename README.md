# 📦 TIAGo Delivery Robot

> Rokey Bootcamp Cobot-3rd Project | 🗓️: 2025.12.22 ~ 2026.01.06 <br>
> Original Repository: [C-2-Organization/tiago-delivery](https://github.com/C-2-Organization/tiago-delivery)

An autonomous indoor delivery robot system for apartment buildings, built with **NVIDIA Isaac Sim** and **ROS2**.

---

## 🌟 Overview

### ❓ Problem Statement

In delivery services, the **"last 100 meters"**—from a building's entrance to each residential unit—presents significant inefficiencies. Delivery drivers often face security hurdles and elevator congestion, especially in high-rise buildings.

### ✅ Solution

We propose a dedicated indoor delivery robot to handle the "last 100m":

1. **Drop-off**: Drivers drop packages at a designated zone.
2. **Autonomous Delivery**: The robot navigates, uses QR to read labels, and delivers to doorsteps.
3. **Efficiency**: This system reduces workload, enhances security

---

## 📁 Repository Structure

```text
Cobot_3/
├── README.md
├── .gitignore
├── dependencies.repos
├── setup.sh
└── ros2_ws/
    ├── external/
    │   ├── pal_urdf_utils/             # Utilities for generating URDF models
    │   ├── play_motion2/               # Pre-defined robot motions (msgs, cli)
    │   ├── pmb2_*/                     # TIAGo mobile base control & navigation
    │   ├── tiago_*/                    # Upper body, arms, sensors, and simulation
    │   ├── tiago_moveit_config/        # MoveIt arm trajectory planning
    │   └── urdf_test/                  # URDF validation tools
    └── src/
        ├── interfaces/                 # Custom message, service, and action definitions
        ├── isaacsim_setup/             # Isaac Sim integration & scene files
        ├── manipulation/               # Arm control algorithms and logic
        ├── navigation/                 # Nav2 parameters and path planning
        ├── orchestrator/               # Mission scenario & state machine
        ├── perception/                 # Object recognition & sensor processing
        └── tiago_isaac_ros2_control/   # Isaac Sim ↔ ROS 2 Control interface
```

---

## ⚙️ Prerequisites

* **OS**: Ubuntu 22.04 (Jammy Jellyfish)
* **GPU**: NVIDIA RTX GPU (RTX 30-series / A-series or higher recommended)
* **NVIDIA Driver**: Version 550.x or higher
* **NVIDIA Isaac Sim**: 5.0.0
* **ROS 2**: Humble Hawksbill
* **Python**: 3.10

---

## 🛠️ Installation

### 1. Clone the Repository

```bash
git clone https://github.com/jmjang08/Cobot_3.git
cd Cobot_3
```

### 2. Run Setup Script

The setup script will fetch all external dependencies listed in `dependencies.repos`.

```bash
chmod +x setup.sh
./setup.sh
```

---

## 🚀 How to Run

Follow these steps to run the simulation and start the TIAGo delivery system.

### 1. Setup Environments

**A. Generate QR Codes**

```bash
cd ~/COBOT_3/ros2_ws/src/isaacsim_setup
python3 make_qr.py
```

**B. Load Isaac Sim World**

1. Launch **NVIDIA Isaac Sim 5.0.0**.
2. Open: `ros2_ws/src/isaacsim_setup/maps/map.usd`

**C. Place QR Codes in Box**

1. Open **Script Editor** in Isaac Sim.
2. Load and **Run**: `ros2_ws/src/isaacsim_setup/insert_qr.py`

**D. Configure Domain ID**

```bash
export ROS_DOMAIN_ID=110
```

> [!TIP]
> **If you are using a custom ID**, search for **"ros2_context"** in the Isaac Sim Stage window and ensure the Domain ID there matches your environment variable.

To verify that the Domain ID is set correctly, run `ros2 topic list` in your terminal. You should see the following topics:

```bash
/clock
/cmd_vel
/gemini2/color/camera_info
/gemini2/color/image_raw
/gemini2/depth/camera_info
/gemini2/depth/image_raw
/joint_command
/joint_states
/odom
/parameter_events
/rosout
/scan_front_raw
/scan_rear_raw
/tf
```

---

### 2. Build the Workspace

```bash
cd ~/COBOT_3/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

### 3. Create map.yaml (Optional)

If you want to generate a new map:

1. **Controller**: `python3 tiago_example_controller.py`
2. **SLAM**: `ros2 launch tiago_cartographer tiago_cartographer.launch.py` <br>
Use the controller to teleoperate the robot so that it can scan all surrounding walls and surfaces.
3. **Save**: `ros2 run nav2_map_server map_saver_cli -f ~/map` <br>
Move the generated map files to: `Cobot_3/ros2_ws/src/isaacsim_setup/maps/`.

### 4. Run Project

Launch the main bringup file:

```bash
ros2 launch orchestrator tiago_bringup.launch.py
```

⚠️ **Performance Note**: If the Isaac Sim frame rate is below **9 FPS**, run the following commands sequentially in separate terminals:

* **After pressing Play in Isaac Sim**:
`ros2 launch receiver_detection perception.launch.py`
* **After reaching the box pickup position**:
`ros2 launch manipulation manipulation.launch.py`
* **After successfully picking up the box**:
`ros2 launch tiago_nav2 nav2.launch.py`
* **Final Orchestrator**:
`ros2 launch orchestrator orchestrator.launch.py`

---

## 🤖 Robot Platform

This project utilizes the **PAL Robotics TIAGo++**:

* **Base**: Omnidirectional mecanum wheels for tight spaces.
* **Arms**: Dual 7-DoF arms for complex manipulation.
* **Sensors**: Gemini2 RGB-D camera & dual laser scanners.

> For more info: [PAL Robotics TIAGo](https://pal-robotics.com/robot/tiago/)

---

## 📜 License

This project is licensed under the **Apache License 2.0**.  
Feel free to use, modify, and distribute this software under the terms of the Apache License.

**⚠️ Important Notes on Dependencies:**

* **tiago_isaac**: This dependency is licensed under **AGPL-3.0**.
* **map.usd**: Redistribution of the original world file is **strictly prohibited** as per the creator's terms.  
  (Source: ["Apartment hallways, elevators and stairs" by orange1718(프레피)](https://www.acon3d.com/ko/product/1000009405))