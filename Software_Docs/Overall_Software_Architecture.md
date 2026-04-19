## Overall Software Architecture

Architecturally, the system follows a modular node-based design coordinated by a supervisor_node finite state machine. During exploration, frontier_node autonomously searches the map for reachable frontiers and sends goals through Nav2. In parallel, marker_mapper_node processes the camera feed to detect station ArUco markers and computes a coarse docking pose in the map frame. Once a station is selected, approach_nav_node sends that coarse goal to Nav2. After the robot reaches the staging point, docking_node takes over and performs final visual servoing using pose estimation and direct velocity commands. Once docking is complete, the supervisor triggers either the stationary or dynamic launcher hardware node. For the dynamic station, a separate dynamic_launch_clearance_node monitors the moving target and publishes launch-clearance signals whenever the dynamic marker re-enters full view. The overall ROS2 node architecture can be found below. It highlights all the topics along with their respective subscribers and publishers.

## Node Architecture Diagram

![ROS2 node architecture diagram](./Images/Node_Architecture.png)


