## Overview of Electrical Subsytem

## Electrical Components

| Component | Purpose in System | Key Specifications | Connections / Interface | Notes |
|---|---|---|---|---|
| **OpenCR** | Main robot controller for TurtleBot3 base and low-level hardware interfacing | ARM Cortex-based control board, ROS-compatible, supports motor and sensor interfacing | Connected to Raspberry Pi; interfaces with TurtleBot3 actuators and sensors | Used for low-level robot motion control and hardware communication |
| **Raspberry Pi 4B+** | Onboard companion computer for running custom ROS 2 nodes and control logic | Quad-core processor, Linux support, GPIO pins, USB, Wi-Fi | Connected to OpenCR, camera, motor driver, and servo through GPIO / USB | Runs higher-level software such as FSM logic, perception integration, and launch control |
| **Raspberry Pi Camera Module 2** | Captures live visual data for ArUco marker detection and station alignment | CSI camera module, supports high-resolution image capture and video streaming | Connected to Raspberry Pi through CSI ribbon cable | Used for computer vision tasks such as marker detection, pose estimation, and docking support |
| **RS360 12V DC Motor** | Drives the launching mechanism for ball delivery | 12 V DC motor, high-speed rotary motion | Connected to L298N motor driver | Provides the mechanical actuation needed to launch ping pong balls |
| **Micro Servo SG90** | Controls angular positioning of for servo arm controlling release of balls into flywheel| 5 V micro servo, approximately 0° to 180° rotation | Connected to Raspberry Pi GPIO with PWM control | Used where precise angular movement is needed, such as opening or releasing a mechanism |
| **L298N Motor Driver** | Drives the DC motor by allowing direction and power control from logic signals | Dual H-bridge motor driver, supports up to 35 V input | Connected to Raspberry Pi GPIO for control signals and to RS360 motor for output | Used to control the 12 V DC motor safely from low-voltage logic signals |

## Electrical and Electronics Diagrams

This is an overview of the various electrical connections. The raspberry pi camera is not drawn in the image.
<img src="Images/connections_diagram.png" width="500" height="300">


## Systems Arcitecture 

This covers both the power architecture as well as the communications architecture.
<img src="Images/sys_archi_diagram.png" width="500" height="300">
