# ArUco-Based Docking Using OpenCV Pose Estimation

## Overview

To achieve autonomous docking, we implemented a custom vision-based docking pipeline using the Raspberry Pi camera, OpenCV ArUco detection, and pose estimation. The camera stream is subscribed to through `/image_raw/compressed`, and the detected marker pose is estimated using `cv2.solvePnP`, with camera intrinsics loaded from a prior calibration file. The final docking node is built as a custom ROS 2 finite state machine that detects a target ArUco marker, evaluates pose stability, performs a one-time coarse correction if needed, and then uses closed-loop fine alignment to dock accurately. The final implementation is reflected in our docking node, which uses `CompressedImage`, `Odometry`, ArUco detection, `solvePnP`, odometry-based turning, and a custom FSM for docking execution. 

## Pose Estimation and Vision Pipeline

Our docking approach is based on estimating the relative pose of the ArUco marker with respect to the robot camera.

### Camera and OpenCV usage

The docking node uses:
- OpenCV ArUco detection with `DICT_4X4_50`
- `cv2.solvePnP` for pose estimation
- a calibrated camera matrix and distortion coefficients
- a known physical marker size of **4 cm**

The 3D marker corner coordinates are defined in the marker frame, and the 2D image corners are detected from the camera frame. `solvePnP` then estimates:
- **`tvec`**: translation of the marker relative to the camera
- **`rvec`**: rotation of the marker relative to the camera

### Meaning of `tvec`

The translation vector was the main quantity used during docking:
- **`tx`**: lateral offset of the marker in the camera frame
- **`tz`**: forward distance from the camera to the marker

These values formed the basis of our final fine docking controller.

In the final version:
- the target **`tx`** was tuned to **3 cm**
- acceptable tolerance on `tx` was **±0.5 cm**
- the target **`tz`** was tuned to **0.21 cm**
- acceptable tolerance on `tz` was **±3 cm** 

### Meaning of `rvec` and heading error

We experimented with using rotational information from pose estimation to improve docking orientation.

Initially, we explored converting `rvec` to Euler angles and reasoning in terms of yaw. However, this introduced extra complexity due to frame conventions, sign interpretation, and instability close to the marker. Instead of directly using Euler yaw from `rvec`, the final version computed a custom **heading error** from the marker normal.

This was done by:
1. converting `rvec` to a rotation matrix using `cv2.Rodrigues`
2. extracting the marker's local **z-axis**, which is the marker normal
3. projecting this normal into the camera frame
4. computing the heading error from the x-z projection

This produced a heading error that was approximately:
- **0 degrees** when the robot camera was facing the marker front-on
- positive or negative when the robot was angled relative to the marker 

---

# Final Custom FSM for Docking

The final docking node was implemented as a custom finite state machine. The states in the final version were:

1. `SEARCHING_FOR_ID`
2. `CHECKING_INITIAL_POSE_STABILITY`
3. `MOVING_FORWARD_FOR_STABLE_POSE`
4. `DECIDE_DOCKING_STRATEGY`
5. `COARSE_ALIGN_ROTATE_1`
6. `COARSE_ALIGN_SHIFT`
7. `COARSE_ALIGN_ROTATE_2`
8. `FINE_ALIGN_AND_DOCK`
9. `FINAL_HEADING_ALIGN`
10. `DONE`

## 1. SEARCHING_FOR_ID

In this state, the robot rotates on the spot while searching for the target ArUco marker. Detection is not accepted immediately. Instead, the system requires the marker to be seen consistently across multiple frames before transitioning.

This prevents false positives and noisy one-frame detections from triggering docking prematurely.

## 2. CHECKING_INITIAL_POSE_STABILITY

Once the target marker is seen consistently, the robot stops and evaluates whether the estimated pose is stable.

A sliding window of recent pose estimates is stored, and standard deviation is computed across both:
- position
- orientation

Only when pose variation falls below configured thresholds is the pose considered stable enough for docking.

This state was critical because ArUco pose estimates can be noisy when:
- the robot is moving
- the marker is near the image edge
- lighting conditions vary
- the camera is too far away

## 3. MOVING_FORWARD_FOR_STABLE_POSE

If the pose is not yet stable, the robot moves slightly forward to obtain a cleaner and larger view of the marker. The idea was that the marker becomes more reliably estimated when it occupies more of the frame and is viewed less obliquely.

The robot then checks pose stability again.

## 4. DECIDE_DOCKING_STRATEGY

This state decides whether coarse alignment is necessary before fine docking.

In the final implementation, this decision was based on the **heading error**, not on `tx`.

If the heading error magnitude was greater than **10 degrees**, the robot executed a single coarse alignment round. Otherwise, it skipped directly to fine docking. 

---

# Re-Iterations of Coarse Alignment

## Initial approach: repeated coarse alignment using `tx`

The initial docking logic used repeated coarse alignment based on horizontal marker offset.

### Initial coarse logic
The robot would:
1. check whether `tx` was large
2. rotate **90 degrees**
3. move sideways by a distance proportional to `tx`
4. rotate back **90 degrees**
5. re-evaluate
6. repeat coarse alignment left or right until `tx` became small enough
7. then switch to a proportional fine controller on `tx` and `tz`

### Why this was initially reasonable
This method was intuitive because:
- `tx` directly reflects lateral misalignment in the camera frame
- a large `tx` suggests the robot is not centred with respect to the marker
- a coarse open-loop side shift can reduce this error before starting fine docking

### Problems observed in test runs
During testing, this repeated coarse approach had several issues:
- it often **overshot**
- repeated side shifts were difficult to tune robustly
- it could oscillate between left and right corrections
- a small change in detection after reacquisition could cause the coarse direction to flip
- repeated coarse corrections made the overall behaviour less predictable

As a result, repeated coarse alignment was not sufficiently robust for reliable competition runs.

---

## Final coarse alignment logic: single coarse correction using heading error

To improve robustness, we changed the coarse logic significantly.

### Main change
Instead of continuously coarse-aligning left or right until `tx` became small, the final design performs **at most one coarse alignment round** in a docking sequence.

The decision to coarse-align is made using the **heading error**.

If the robot is visibly angled relative to the marker, it is often more effective to correct this geometry directly than to repeatedly react to `tx` alone.

### Final coarse alignment sequence
If heading error > 10 degrees:
1. stop the robot
2. compute heading error from the marker normal
3. compute a lateral shift estimate using:

\[
\text{shift distance} = \tan(\theta)\times tz
\]

where:
- \(\theta\) is the heading error
- \(tz\) is the forward marker distance

4. rotate by **90 degrees**
5. move forward by the computed shift distance
6. rotate back using an angle based on the original heading error
7. mark coarse alignment as completed for that docking sequence
8. proceed to re-acquire stable pose and continue docking

### Why this makes geometric sense
The heading error represents how far the robot is angled relative to the marker plane. In the x-z plane, this can be modelled as a right triangle:
- forward distance is approximated by `tz`
- angular misalignment is \(\theta\)
- the estimated lateral correction is therefore approximately:

\[
x \approx tz \tan(\theta)
\]

This gives a more geometry-aware correction than simply shifting by a value derived from `tx`.

### Why we only do this once
Single coarse correction was chosen because:
- it was more stable than repeated open-loop shifts
- it reduced the risk of oscillating coarse behaviour
- it simplified tuning
- after one large geometric correction, the remaining error could be handled more safely by fine closed-loop control

This final version was more predictable in practice. 

---

# Re-Iterations of Fine Alignment

## Initial fine alignment approach
Initially, after coarse alignment, we used a proportional controller on:
- `tx`
- `tz`

This made sense because:
- `tx` controls lateral visual centring
- `tz` controls stopping distance from the target

### Basic idea
- angular velocity was generated from `tx`
- linear velocity was generated from `tz`
- if `tx` was still large, the robot rotated first before moving forward

This worked reasonably well but still required several tuning iterations.

---

## Attempt to include heading error in continuous PID
At one stage, we also experimented with adding heading error into the fine controller.

The motivation was that even when `tx` and `tz` were near target values, the robot could still be slightly slanted relative to the marker. We therefore tested adding a heading-based correction term during docking.

### Why this was explored
A robot can appear centred in translation while still not being properly square to the docking target. This meant:
- `tx` could look acceptable
- `tz` could look acceptable
- but the final orientation could still be off

### Why it was later removed
In practice, adding heading error continuously into the fine PID introduced more complexity:
- the marker pose changed rapidly at close range
- heading estimates became noisy
- correction direction was sensitive to sign convention
- the controller became harder to tune
- the robot could oscillate or over-correct near the dock

Because of this, the final fine docking controller was simplified back to using only:
- `tx`
- `tz`

Heading correction was separated conceptually from the main translational docking loop.

---

# Final Fine Alignment and Docking Strategy

The final fine docking state used PID-style proportional control on **`tx`** and **`tz`** only.

## Final control logic
- **angular velocity** was computed from `tx`
- **linear velocity** was computed from `tz`
- if `tx` was still large, the robot rotated first and did not move forward
- once `tx` was within an acceptable band, forward motion was allowed

This reduced the chance of drifting forward while still strongly misaligned laterally.

## Final fine docking targets
The final tuned docking targets were:
- **target `tx` = 0.03 m**
- **target `tz` = 0.21 m**

The final success tolerances were:
- **`tx` tolerance = ±0.005 m**
- **`tz` tolerance = ±0.03 m**

## Final proportional gains
The gains used in the final fine docking implementation were:

`self.kp_x = 2.5`

# Final Parameters Tuned

The final version required tuning across several categories.

### Pose filtering and detection stability

`required_detection_frames`
`pose_window_size`
`pos_std_thresh`
`angle_std_thresh_deg`

These affected:

how confidently the marker was accepted
how stable the pose estimate had to be before docking
how often false or noisy detections triggered state transitions

### Fine docking targets

`target_x_m`
`target_x_tol_m`
`target_z_m`
`target_z_tol_m`
`self.kp_z = 0.5`

These determined:

how centred the robot had to be
how far from the marker it should stop
what positional accuracy counted as successful docking

### Fine docking controller

`kp_x`
`kp_z`
`max_linear_speed`
`max_angular_speed`
`align_first_x_thresh_m`

These directly affected:

how strongly the robot corrected lateral error
how aggressively it approached forward
when it should rotate first instead of moving forward

### Coarse alignment tuning

`coarse_heading_thresh_deg`
`coarse_shift_max_m`
`coarse_forward_speed`
`coarse_rotate_speed`

These controlled:

when coarse correction should trigger
how large the sideways correction was allowed to be
how fast the robot executed the open-loop motion

### Final heading correction tuning

`kp_heading_final`
`max_heading_align_speed`
`final_heading_tol_deg`
`final_heading_max_step_deg`

These were tuned while experimenting with final heading refinement after docking.

All of these tunable values appear in the final docking node configuration section.
