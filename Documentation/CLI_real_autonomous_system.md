
# Requirements

- **ROS 2 Humble**
- **TurtleBot3 packages**
- **Cartographer SLAM**
- **Nav2 Packages**
- **Gazebo (for simulation)**

# TurtleBot3 Setup Chcklist

Ensure your TurtleBot3 is:

- Properly calibrated (motors & sensors)
- Connected to the same network as your PC
- Configured with correct ROS 2 environment variables:
ROS_DOMAIN_ID
ROS_WORKSPACE_NAME
Right TurtleBot3 model
Other necessary parameters


# How to fully run the software on a TurtleBot3 system!

## Setup on Raspberry Pi

1. **Connect to Raspberry Pi:**

```bash
ssh username@<ip-address>
```

2. **Git clone the repoistory into the src folder of your ros2 workspace on the raspberry pi:**

Ours is called turtlebot3_ws. Please replace it with your own ros2 workspace name.

```bash
cd turtlebot3_ws/src
git clone git@github.com:jitisha-a/cde2310_custom_robot_stack.git
```
3. **Exit src and build the newly installed package. Then source your bash:**

```bash
cd ..
colcon build --packages-select cde2310_custom_robot_stack
source install/setup.bash
```

4. **rosbu / Bring up ROS on the TurtleBot3:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

5. **Launch RPI Camera Node:**

Ensure your settings match your camera calibration specs.

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480] \
  -p time_per_frame:=[1,30]
```

6. **Launch the flywheel-launcher node for the stationary station:**
```bash
ros2 run cde2310_custom_robot_stack stationary_launcher_hw_node.py
```

7. **Launch the flywheel-launcher node for the dynamic station:**
```bash
ros2 run cde2310_custom_robot_stack dynamic_launcher_hw_node.py
```

## Setup on Remote Dekstop

1. **Git clone the repoistory into the src folder of your ros2 workspace on the raspberry pi:**

Ours is called turtlebot3_ws. Please replace it with your own ros2 workspace name.

```bash
cd turtlebot3_ws/src
git clone git@github.com:jitisha-a/cde2310_custom_robot_stack.git
```
2. **Exit src and build the newly installed package. Then source your bash:**

```bash
cd ..
colcon build --packages-select cde2310_custom_robot_stack
source install/setup.bash
```

3. **rslam / Launch R-SLAM (Cartographer):**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

4. **Launch Nav2:**

```bash
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/YOUR_USERNAME/turtlebot3_ws/src/cde2310_custom_robot_stack/config/nav2_params.yaml
```

4. **Launch the Full ROS2 System**

```bash
ros2 launch cde2310_custom_robot_stack system.launch.py
```
