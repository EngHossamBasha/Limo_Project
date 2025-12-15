# LIMO COBOT Project

A ROS Noetic simulation of the AgileX LIMO mobile robot with MyCobot 280 robotic arm for pick-and-place applications.

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange)
![License](https://img.shields.io/badge/License-MIT-green)

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Troubleshooting](#troubleshooting)
- [Team](#team)

## 🎯 Overview

This project combines the AgileX LIMO differential drive mobile robot with a MyCobot 280 6-DOF robotic arm, creating a mobile manipulation platform suitable for:
- Autonomous navigation
- Object manipulation
- Pick-and-place operations
- Research and education

## ✨ Features

- **Gazebo Simulation**: Full physics simulation with realistic sensor models
- **MoveIt Integration**: Motion planning for the robotic arm
- **Keyboard Teleoperation**: Manual control of the mobile base
- **Multiple Sensors**: LiDAR, depth camera, IMU
- **Navigation Ready**: Pre-configured for SLAM and autonomous navigation

## 📦 Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 LTS
- **ROS**: ROS Noetic (Full Desktop Install)
- **RAM**: Minimum 8GB (16GB recommended)
- **GPU**: Dedicated GPU recommended for Gazebo

### Required ROS Packages

Install the following packages before building the workspace:

```bash
# Update package list
sudo apt update

# Install ROS Noetic (if not already installed)
sudo apt install ros-noetic-desktop-full

# Install required dependencies
sudo apt install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    ros-noetic-moveit \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-xacro \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-gmapping \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner \
    ros-noetic-global-planner \
    ros-noetic-teb-local-planner

# Install Python dependencies
pip3 install numpy
```

## 🚀 Installation

### Step 1: Create Workspace (if new installation)

```bash
# Create catkin workspace
mkdir -p ~/limo_cobot_ws/src
cd ~/limo_cobot_ws/src
```

### Step 2: Clone Repository

```bash
# Clone the repository
git clone https://github.com/EngHossamBasha/Limo_Project.git .

# Note: Clone directly into src/ folder (notice the dot at the end)
```

### Step 3: Build the Workspace

```bash
# Go to workspace root
cd ~/limo_cobot_ws

# Build
catkin_make

# Source the workspace
source devel/setup.bash
```

### Step 4: Add to Bashrc (Recommended)

```bash
# Add workspace to bashrc for automatic sourcing
echo "source ~/limo_cobot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🎮 Usage

### Launch Simulation with MoveIt

Open a terminal and run:

```bash
roslaunch limo_project demo_gazebo.launch
```

This will start:
- Gazebo with the LIMO COBOT robot
- RViz with MoveIt motion planning interface
- All necessary controllers

### Keyboard Teleoperation

In a **new terminal**, run:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Controls:**
| Key | Action |
|-----|--------|
| `i` | Move Forward |
| `,` | Move Backward |
| `j` | Turn Left |
| `l` | Turn Right |
| `k` | Stop |
| `u` | Forward + Left |
| `o` | Forward + Right |
| `q/z` | Increase/Decrease Speed |

**Important:** Click on the teleop terminal to focus it before pressing keys!

### Using MoveIt for Arm Control

1. In RViz, go to the "MotionPlanning" panel
2. Drag the interactive markers to set target pose
3. Click "Plan" to compute trajectory
4. Click "Execute" to move the arm

## 📁 Package Structure

```
src/
├── Limo_Project/           # Main project package (custom launches)
│   ├── launch/
│   │   ├── demo_gazebo.launch      # Main simulation launch
│   │   └── gazebo.launch           # Gazebo-specific launch
│   ├── param/                      # Configuration parameters
│   ├── rviz/                       # RViz configurations
│   └── worlds/                     # Gazebo world files
│
├── limo_description/       # LIMO robot URDF and meshes
│   ├── urdf/
│   │   └── limo_base.urdf          # Robot model
│   └── meshes/                     # 3D mesh files
│
├── limo_gazebo_sim/        # Gazebo simulation configs
│   ├── config/                     # Controller configs
│   ├── launch/                     # Simulation launches
│   └── worlds/                     # World files
│
├── mycobot_description/    # MyCobot arm URDF and meshes
│   ├── urdf/                       # Arm model files
│   └── meshes/mycobot_280/         # Arm mesh files (.dae)
│
└── limo_cobot_moveit/      # MoveIt configuration
    ├── config/                     # MoveIt configs
    │   ├── kinematics.yaml
    │   ├── joint_limits.yaml
    │   └── ompl_planning.yaml
    └── launch/                     # MoveIt launches
```

## 🔧 Troubleshooting

### Common Issues

**1. "Package not found" error**
```bash
# Make sure workspace is sourced
source ~/limo_cobot_ws/devel/setup.bash

# Verify package is found
rospack find limo_project
```

**2. Gazebo crashes or robot doesn't appear**
```bash
# Kill any existing Gazebo processes
killall gzserver gzclient

# Try launching again
roslaunch limo_project demo_gazebo.launch
```

**3. Robot doesn't move with teleop**
- Make sure to click on the teleop terminal to focus it
- Check if `/cmd_vel` topic is being published:
  ```bash
  rostopic echo /cmd_vel
  ```
- Ensure Gazebo simulation is not paused (check Play button)

**4. MoveIt planning fails**
- Check if the arm is in collision with itself or the environment
- Try a different planner (RRTstar, PRM, etc.)
- Increase planning time in MoveIt panel

**5. Missing mesh files error**
- All meshes are included in this repository
- If errors persist, rebuild the workspace:
  ```bash
  cd ~/limo_cobot_ws
  catkin_make clean
  catkin_make
  ```

### Checking System Health

```bash
# List all running nodes
rosnode list

# Check if controllers are loaded
rosservice call /controller_manager/list_controllers

# Monitor cmd_vel topic
rostopic hz /cmd_vel
```

## 👥 Team

- **Hossam Basha** - [GitHub](https://github.com/EngHossamBasha)
- **Karim** - Project Developer
- Team Members

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- AgileX Robotics for the LIMO platform
- Elephant Robotics for MyCobot
- ROS and MoveIt community
- Our project supervisor

---

**Note:** This project was developed as part of our robotics course. For questions or issues, please open a GitHub issue.
