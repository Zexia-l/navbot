# 🤖 NavBot: ROS 2 Autonomous Navigation Stack

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&style=for-the-badge)
![PlatformIO](https://img.shields.io/badge/Firmware-PlatformIO-orange?logo=platformio&style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active%20Development-yellow?style=for-the-badge)

> **NavBot** is a modular robotics project exploring autonomous navigation using **ROS 2 Humble** and the **Navigation2 (Nav2)** stack. It bridges high-level path planning with low-level hardware control using **micro-ROS** on an Arduino Due, featuring full simulation support in Gazebo.

---

## 📖 Table of Contents
- [Overview](#-overview)
- [Key Features](#-key-features)
- [Hardware Specifications](#-hardware-specifications)
- [Installation](#-installation)

---

## 🔭 Overview

This project aims to create a robust mobile robot capable of SLAM (Simultaneous Localization and Mapping) and autonomous navigation. It demonstrates a hybrid development approach where the software stack runs identically on both the **Gazebo simulator** and the **physical robot**.

The core innovation lies in the **custom hardware interface** built with `micro-ROS` and `ros2_control`. Instead of a simple bridge, the Arduino Due acts as a smart node, managing 4 independent motors and publishing synchronized joint states to the ROS 2 ecosystem.

## Preview

### Robot & SLAM Preview

![image](https://github.com/Zexia-l/navbot/blob/main/images/image_preview_realrobot.png)

![image](https://github.com/Zexia-l/navbot/blob/main/images/image_rviz_slam.png)

---

## ✨ Key Features

* **ROS 2 Nav2 Integration:** Utilization of the standard Navigation2 stack for global/local planning and behavior trees (*In Progress*).
* **Micro-ROS Firmware:** Custom firmware for **Arduino Due** using PlatformIO to handle real-time motor control loops.
* **ros2_control Interface:** Standardized `diff_drive_controller` implementation. The firmware subscribes to `cmd_vel` (converted to joint commands) and publishes `JointState` for 4 separate motors.
* **SLAM Toolbox:** Real-time 2D mapping and localization using **RPLidar A1**.
* **Hybrid Simulation:** Accurate **XACRO/URDF** descriptions allowing seamless switching between Gazebo simulation and real hardware.

---

## ⚙️ System Architecture

### Software Stack (ROS 2 Humble)
* **Navigation:** Nav2 (Planner, Controller, Recoveries).
* **Mapping:** `slam_toolbox` (async mode).
* **Control:** `ros2_control`, `diff_drive_controller`, `joint_state_broadcaster`.

### Firmware Logic (Arduino Due)
The micro-ROS agent performs the following cycle:
1.  **Subscribe** to Joint Commands (Target Velocity for 4 motors).
2.  **Execute** PID/Open-loop control on DC Motors via PWM.
3.  **Read** Encoders from 4 motors.
4.  **Publish** `JointState` (Position/Velocity) back to ROS 2.

> **Note on Kinematics:** The robot currently uses **Inverse Differential Drive Kinematics** logic applied to the 4-motor chassis, even though the physical platform supports Mecanum wheels.

---

## 🔧 Hardware Specifications

### Robot Physical Constants
| Parameter | Value | Unit |
| :--- | :--- | :--- |
| **Wheel Diameter** | 0.03 | Meters |
| **Wheel Base** | 0.34 | Meters |
| **Encoder Resolution** | 2000 | Ticks/Rev |
| **PWM Deadzone** | 70 | 0-255 |

### Pinout Configuration (Arduino Due)
The firmware is configured for a 4-wheel setup.

| Motor | Location | RPWM | LPWM | REN | LEN | Enc A | Enc B |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Motor 0** | Front Left | 3 | 2 | 24 | 22 | 27 | 29 |
| **Motor 1** | Front Right | 5 | 4 | 28 | 26 | 31 | 33 |
| **Motor 2** | Back Left | 6 | 7 | 30 | 32 | 35 | 37 |
| **Motor 3** | Back Right | 8 | 9 | 34 | 36 | 39 | 41 |

* **Lidar:** RPLidar A1 (USB Serial)
* **MCU:** Arduino Due (Native USB Port)

---

## 📥 Installation

### Prerequisites
* Ubuntu 22.04 LTS
* ROS 2 Humble Hawksbill
* PlatformIO (VSCode Extension or CLI)
