# ROS1 Mobile Manipulator Platform  
**Differential-Drive Robot with 5-DOF Robotic Arm – Full Stack from Hardware to Motion Planning**

A complete, production-grade robotics system integrating a mobile base and a 5-DOF manipulator, developed under **ROS Noetic** on Ubuntu 20.04.

This repository contains the entire development stack:
- Custom **hardware designs** (KiCad) for servo power distribution and current limiting
- **Low-level embedded drivers** in C++ (UART-based motor control, PWM generation, BLDC/servo feedback)
- **ROS hardware interface** with real-time joint state publishing and trajectory execution
- **MoveIt! motion planning** (OMPL, CHOMP, STOMP pipelines)
- Full **Gazebo simulation** and **RViz visualization**
- Comprehensive **teleoperation** (jogging, joypad, scripted routines)

---

### System Overview

| Component                  | Technology / Implementation                                      |
|----------------------------|------------------------------------------------------------------|
| Mobile Base                | Differential-drive / Ackermann steering, UART motor controllers  |
| Manipulator                | 5-DOF serial arm + parallel gripper                              |
| Motion Planning            | MoveIt! with multiple planners (RRTConnect, PRM, CHOMP, STOMP)   |
| Simulation                 | Gazebo 11 with physics, sensors, and plugin support              |
| Real-Time Control          | Custom C++ `robot_hw` interface (ROS Control compatible)         |
| Teleoperation              | Cartesian/space jogging, joypad mapping, scripted pick & place   |
| Power Electronics          | Custom KiCad shields: multi-channel servo current limiting       |
| Communication              | UART master-slave protocol with CRC8, real-time voltage logging  |

---

### Repository Structure
HW/                              → KiCad hardware designs
Servo_Motor_Shield_v1/       → Final servo power shield
Proto_Servo_Current_Limiter/ → Prototype current-limiting board
ROS/arm_and_chassis_ws/src/
s3a_main/                    → URDF, robot hardware interface, controllers
s3a_moveit_config/           → MoveIt! configuration and planning pipelines
arm_teleop/                  → Jogging and joypad teleoperation
common_teleop/               → Shared routines, pick-and-place sequences
wc_teleop/                   → Mobile base teleoperation
s3a_serial_main/             → UART motor controller master node
SW/Driver/motor_ctrl/            → Embedded C++ drivers (PWM, BLDC, servo)
SW/Test/                         → Unit and integration tests (waf build system)
text---

### Quick Start (Real Hardware & Simulation)

```bash
# Clone and build
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/cxxx1828/ROS1-Robot-Car-and-Robot-Arm.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# Real hardware launch
roslaunch s3a_main main.launch

# Simulation + MoveIt!
roslaunch s3a_moveit_config demo_gazebo.launch

# Teleoperation
roslaunch arm_teleop jog_teleop.launch          # Arm jogging
roslaunch wc_teleop manual_teleop.launch        # Drive base
roslaunch common_teleop routines_teleop.launch  # Pick & place demo

Key Demonstrations

Mobile manipulation with mounted arm
Precision pick-and-place (nail picking routine)
Real-time current/voltage monitoring via UART
Seamless switching between simulation and real hardware


Target Applications

Academic research in mobile manipulation
Industrial automation prototyping
Robotics education (ROS, MoveIt!, embedded systems)
Portfolio projects for robotics/software engineering roles


License
**
MIT License — free for academic, research, and commercial use.
