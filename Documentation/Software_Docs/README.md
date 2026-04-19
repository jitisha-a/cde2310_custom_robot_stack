# Software Documentation

This `Software_Docs` folder contains the software and firmware documentation for our autonomous robot system. It brings together the high-level system design, software architecture, navigation logic, docking and computer vision implementation, and firmware and launching logic used across the project.

The documentation is split into separate Markdown files so that each major part of the system can be explained clearly and in enough detail, without making one file too long or difficult to navigate. This also makes it easier to update specific parts of the documentation as the system evolves.

## Documentation Structure

### 1. [Overall Software Architecture](./Overall_Software_Architecture.md)
This file gives the high-level design of the autonomous system. It explains how the full software stack is structured, how the major nodes and subsystems interact, and how responsibilities are divided across the robot and external compute environment.

### 2. [Autonomous Navigation](./auto_nav.md)
This file documents the autonomous navigation pipeline. It covers map-based navigation, exploration and movement logic, and the software flow used for robot motion during the mission.

### 3. [Docking and OpenCV Implementation](./docking_%26_OpenCV_implementation.md)
This file explains the vision-based docking system. It covers the use of OpenCV and ArUco markers, pose estimation, docking control logic, and the design iterations made to improve docking reliability and alignment accuracy.

### 4. [Firmware and Launching Logics](./firmware_%26_launching_logics.md)
This file documents the firmware-side logic and the launching subsystem. It includes the control logic for ball launching, actuator behaviour, and how firmware and software components work together during the final execution process.
