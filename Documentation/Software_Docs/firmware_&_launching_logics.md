# Firmware and Launching Logic Documentation

This document describes the firmware-side launching implementation used for both the stationary and dynamic delivery stations. The launcher system is split into separate ROS 2 nodes so that perception, hardware control, and mission-level triggering remain modular and easier to debug. The implementation is based on three main files: the desktop-side dynamic launch clearance node, the Raspberry Pi dynamic launcher hardware node, and the Raspberry Pi stationary launcher hardware node.

---

## 1. System Overview

The launching subsystem was designed differently for the two stations:

- **Stationary station:** fully time-based launching on the Raspberry Pi.
- **Dynamic station:** perception-assisted launching, where a desktop vision node decides when the moving target is properly visible and sends a clearance signal to the Raspberry Pi hardware node.

This split was necessary because the dynamic station required synchronization with a moving receptacle, while the stationary station could be handled with a fixed timed sequence. The dynamic implementation therefore uses a handshake between:
- a **desktop vision node** that detects ArUco marker visibility, and
- a **Pi hardware node** that spins the flywheel motor and actuates the servo only when clearance is received. :contentReference[oaicite:3]{index=3} :contentReference[oaicite:4]{index=4}

---

## 2. Hardware Used by the Launcher Nodes

Both hardware nodes use the same GPIO-controlled launcher mechanism:

- **Servo motor** for releasing one ball at a time
- **DC motor** for spinning the launcher / flywheel
- **L298 motor driver** for motor direction and PWM speed control
- **Raspberry Pi GPIO PWM** for both servo and flywheel actuation

### GPIO Mapping

- `SERVO_PIN = 12`
- `ENA = 18`
- `IN1 = 23`
- `IN2 = 24`

These pins are configured in BCM mode and initialized during node startup. Both launcher nodes also initialize the motor in a stopped state and move the servo into its hold position before waiting for commands. :contentReference[oaicite:5]{index=5} :contentReference[oaicite:6]{index=6}

---

## 3. Common Launch Mechanism Behavior

Although the stationary and dynamic launchers differ in sequencing logic, they share the same basic actuation behavior.

### 3.1 Flywheel Motor Control

The DC motor is driven in reverse using:

- `IN1 = LOW`
- `IN2 = HIGH`

Motor speed is controlled through PWM on the enable pin. A helper function clamps duty cycle values to the valid range of 0 to 100%. :contentReference[oaicite:7]{index=7} :contentReference[oaicite:8]{index=8}

### 3.2 Servo Control

The servo uses 50 Hz PWM. The angle is converted to duty cycle using:

`duty = 2 + (angle / 18.0)`

Two main servo angles are used:

Hold angle: keeps the next ball in place
Launch angle: pushes/releases one ball into the spinning launcher

After commanding the servo, the PWM signal is briefly applied, then disabled to avoid continuously driving the servo.

### 3.3 Abort-Aware Delays

Both hardware nodes use a helper function that sleeps in small increments while checking whether an abort has been requested. This makes the system safer and prevents the node from becoming unresponsive during long waits.

---

## 4. Stationary Launcher Firmware

File: stationary_launcher_hw_node.py

### 4.1 Purpose

This node handles all hardware control for launching three balls at the stationary station. Since the target is fixed, no visual timing feedback is needed. Once the launch command is received, the node executes the entire sequence autonomously using timed delays.

### 4.2 ROS Interface

Subscriber
/launch_stationary_cmd (std_msgs/Bool)
Publisher
/launch_done (std_msgs/Bool)

When a True message is received on /launch_stationary_cmd, the node starts the stationary launch sequence, unless one is already in progress. When all three balls have been launched successfully, the node publishes True on /launch_done.

### 4.3 Launch Sequence Logic

The stationary launch is executed in a background thread so the node can continue spinning while the launch sequence runs.

Sequence:
- Receive /launch_stationary_cmd = True
- Reject the command if a launch is already in progress
- Start a background thread running run_launch_sequence()
- Spin the motor in reverse
- Ramp motor speed upward in steps
- Wait for the flywheel to spin up
- Launch ball 1
- Wait for a short fixed delay
- Launch ball 2
- Wait for a longer fixed delay
- Launch ball 3
- Stop the motor
- Publish /launch_done = True

This design was suitable for the stationary station because the timing between launches could be predefined and did not depend on external perception feedback.

### 4.4 Stationary Timing Parameters

The main timing and actuation parameters are:

`RAMP_STEP = 10`
`RAMP_STEP_DELAY = 0.2`
`MOTOR_SPINUP_WAIT = 3.0`
`DELAY_AFTER_FIRST = 2.0`
`DELAY_AFTER_SECOND = 8.0`
`Max_duty = 90`
`SERVO_HOLD_ANGLE = 130`
`SERVO_LAUNCH_ANGLE = 180`
`SERVO_MOVE_TIME = 0.5`
`SERVO_RETURN_SETTLE = 0.2`

These values determine how aggressively the flywheel spins up and how much time is allowed between launches for the mechanism to recover and for the next ball to settle.

### 4.5 Notes on Design

A few key design choices in the stationary launcher are worth noting:

>> The launch process is fully deterministic once triggered.
>> A threaded implementation is used so the ROS node remains responsive.
>> The motor is always stopped in both normal completion and cleanup paths.
>> Duplicate commands are ignored while a launch is already running.

This keeps the node simple and robust for a fixed target station.

-- 

## 5. Dynamic Launcher Firmware

The dynamic launcher is split into two cooperating nodes:

`DynamicLaunchClearanceNode` on the desktop
`DynamicLauncherHwNode` on the Raspberry Pi

This split was necessary because the moving target cannot be launched into using fixed time delays alone. Instead, launch permission must be generated only when the target marker is fully and reliably visible.

--

## 6. Dynamic Launch Clearance Node

File: dynamic_launch_clearance_node.py

### 6.1 Purpose

This node runs on the desktop and monitors the camera stream during dynamic launching. It detects ArUco marker ID 29 and publishes a clearance signal only when the marker is confirmed to be fully visible in the frame. This acts as a perception gate for each ball launch.

### 6.2 ROS Interface

Subscribers
/robot_mode (std_msgs/String)
/image_raw/compressed (sensor_msgs/CompressedImage)
Publisher
/dynamic_launch_clear (std_msgs/Bool)

The node only performs image-based checking while the robot mode is LAUNCH_DYNAMIC. Outside this mode, it resets its internal visibility state and does not publish launch clearances.

### 6.3 Marker Detection Logic

The node uses OpenCV ArUco detection with:

`dictionary: DICT_4X4_50`
`target marker ID: 29`

The compressed image is first decoded into an OpenCV frame. The frame is converted to grayscale, and cv2.aruco.detectMarkers() is used to locate markers. The node then checks whether marker 29 exists among the detections.

### 6.4 “Fully Visible” Check

The node does not simply check whether the marker is detected. It checks whether all four corners of marker 29 are inside the image bounds with a pixel margin:

`full_frame_margin_px = 5`

If any corner lies too close to the image boundary, the marker is treated as not fully visible. This prevents sending launch clearance when the moving receptacle is only partially visible at the edge of the frame.

## 6.5 Debouncing and Rising-Edge Logic

A major challenge with the dynamic station is that the marker may flicker in and out of view because of motion, blur, or partial occlusion. To avoid false triggers, the node uses frame-based debouncing:

`frames_required_visible = 4`
`frames_required_not_visible = 4`

The node maintains:

`visible_streak`
`not_visible_streak`

A clearance is published only when:

the marker has been fully visible for at least 4 consecutive frames, and
it was previously considered not currently visible

This makes the logic effectively rising-edge based. It does not continuously publish while the marker remains in frame. Instead, it publishes only when the marker newly reappears. Similarly, it only resets the “currently visible” state after 4 consecutive not-visible frames.

### 6.6 Cooldown Logic

After publishing a clearance, the node enters a cooldown period:

`clearance_cooldown_sec = 5.0`

During this time, the node ignores further checks and publishes nothing. This prevents accidental double-triggering from a marker that remains visible right after a launch.

### 6.7 Clearance Count Limiting

The node also keeps count of how many clearances have been issued:

`max_clearances = 3`

Once three clearances have been published, the node stops issuing more. This matches the requirement of launching exactly three balls.

### 6.8 Dynamic Clearance Flow

The effective logic is:

- Wait until robot mode becomes LAUNCH_DYNAMIC
- Reset internal visibility and cooldown state
- Decode each compressed camera image
- Detect ArUco marker 29
- Check if the marker is fully inside the frame
- Debounce visibility over multiple frames
- On a rising edge of confirmed visibility, publish /dynamic_launch_clear = True
- Enter cooldown
- Wait for the marker to disappear and reappear before issuing the next clearance

This means each ball launch is tied to a fresh confirmed reappearance of the dynamic target marker.

--

## 7. Dynamic Launcher Hardware Node

File: dynamic_launcher_hw_node.py

### 7.1 Purpose

This node runs on the Raspberry Pi and controls the physical launching hardware for the dynamic station. Unlike the stationary launcher, it does not decide when to fire based on fixed delays. Instead, it waits for clearance messages from the desktop perception node.

### 7.2 ROS Interface

Subscribers
/launch_dynamic_cmd (std_msgs/Bool)
/dynamic_launch_clear (std_msgs/Bool)
Publisher
/launch_done (std_msgs/Bool)

The node starts the overall dynamic launch sequence when it receives /launch_dynamic_cmd = True. It then waits for up to three separate clearance signals on /dynamic_launch_clear, launching one ball for each valid clearance.

### 7.3 Internal State Variables

The dynamic launcher uses the following state variables:

is_launching
abort_requested
awaiting_launch
ball_count
max_balls = 3

These variables ensure the node only launches when appropriate and does not react to stray or duplicated signals.

### 7.4 Dynamic Launch Sequence

When the node receives /launch_dynamic_cmd = True:

It checks whether a launch is already in progress
It resets the abort flag
It sets is_launching = True
It sets awaiting_launch = True
It resets ball_count = 0
It starts the motor in reverse
It applies the configured motor duty cycle
It waits for clearance signals from the desktop node

Each time a valid /dynamic_launch_clear = True is received:

The node checks that it is currently launching
It checks that it is waiting for the next launch
It checks that fewer than 3 balls have been fired
It temporarily disables additional launch acceptance
It increments ball_count
It actuates the servo once to launch one ball
If 3 balls have been launched, it finishes the sequence
Otherwise, it re-enters the waiting state for the next clearance

This structure cleanly separates sequence ownership from launch timing authority. The Pi owns the launcher hardware, but the desktop vision node owns the timing decision.

### 7.5 Dynamic Hardware Parameters

Main parameters in the dynamic hardware node include:

`MOTOR_DUTY = 90`
`MOTOR_PWM_FREQ = 1000`
`SERVO_PWM_FREQ = 50`
`SERVO_HOLD_ANGLE = 130`
`SERVO_LAUNCH_ANGLE = 180`
`SERVO_MOVE_TIME = 0.5`
`SERVO_RETURN_SETTLE = 0.2`
`max_balls = 3`

These values determine the flywheel speed and the servo release motion for each ball.

### 7.6 Completion Behavior

When all three balls have been launched:

the motor is stopped
is_launching is cleared
awaiting_launch is cleared
/launch_done = True is published

If an abort or failure occurs during launch, the node stops the sequence and does not publish completion. Cleanup also ensures that PWM outputs are stopped and GPIO is released properly on shutdown.

--

## 8. End-to-End Dynamic Launch Handshake

The dynamic launch mechanism can be summarized as the following handshake between the desktop and Pi:

FSM / supervisor
    -> publishes /launch_dynamic_cmd = True
        -> DynamicLauncherHwNode starts motor and waits
            -> DynamicLaunchClearanceNode monitors camera stream
                -> marker 29 becomes fully visible and stable
                    -> publishes /dynamic_launch_clear = True
                        -> DynamicLauncherHwNode launches one ball
                            -> waits for next clearance
                                -> repeats until 3 balls launched
                                    -> publishes /launch_done = True

This approach was more robust than a purely timed dynamic launch, because it synchronizes each individual launch to actual target visibility instead of relying on estimated motion timing alone.

--

## 9. Why the Two Launch Strategies Were Different

The main reason for implementing two different launch strategies was the nature of the mission targets:

Stationary station
target location is fixed
launch timing can be predetermined
simple timed sequencing is sufficient
Dynamic station
target moves in and out of a visible launch window
timed launching alone is unreliable
launch permission must depend on live visual confirmation

Because of this, the stationary launcher is a self-contained timed firmware sequence, whereas the dynamic launcher is a perception-triggered hardware execution sequence.

-- 

## 10. Key Design Decisions

### 10.1 Separation of perception and hardware

The desktop handles image processing, while the Pi handles GPIO-level actuation. This keeps the hardware node lightweight and allows vision logic to evolve independently.

### 10.2 Rising-edge based dynamic clearances

The dynamic clearance node only triggers when the marker newly reappears after being absent, instead of continuously firing while it remains visible. This avoids duplicate launches.

### 10.3 Debounce and cooldown protection

Using both consecutive-frame checks and a fixed cooldown made the dynamic launch logic much more stable under noisy visual conditions.

### 10.4 Shared /launch_done completion signal

Both launcher implementations publish the same completion topic. This makes integration with the higher-level FSM cleaner, since the rest of the system only needs to know when launching has finished.

### 10.5 Safe cleanup

Both hardware nodes stop PWM, stop the motor, and call GPIO.cleanup() during destruction, reducing the risk of leaving actuators energized after shutdown.
