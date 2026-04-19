# Camera Calibration

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

6. **In a new terminal, run the calibration file:**

Ensure you follow the calibration instructions. Hold the checkerboard in front of the camera from 20 different angles based on the timings printed to the terminal. This step would be easier if your RPI has a GUI OS.

```bash
cd turtlebot3_ws/src/cde2310_custom_robot_stack/Documentation/testing_codes/camera_calibration.py
```

---

# AruCo Pose Estimation Accuracy testing

1. Repeat the setup steps 1-3 on the raspberry pi onto your remote desktop
2. Run the following file

```bash
cd turtlebot3_ws/src/cde2310_custom_robot_stack/Documentation/testing_codes/pose_estimation_tester.py
```
---

# Gazebo Simulation Navigation Testing

1. **Launch the Gazebo Simulation:**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **Start SLAM with Cartographer:**

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

4. **Launch Nav2:**

```bash
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/YOUR_USERNAME/turtlebot3_ws/src/cde2310_custom_robot_stack/config/nav2_params.yaml
```

4. **Run the frontier node**

```bash
ros2 run cde2310_custom_robot_stack frontier_node.py
```
