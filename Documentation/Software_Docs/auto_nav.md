# Navigation Algorithm

## Overview

The navigation system is responsible for autonomously exploring an unknown environment, detecting target stations, and navigating to them for docking and launching. It is built on top of **Nav2** (the ROS2 Navigation Stack) and is composed of three main components:

1. **Nav2** — handles path planning, local control, and obstacle avoidance
2. **Frontier Node** (`frontier_node.py`) — drives autonomous exploration by detecting and navigating to unexplored regions
3. **Approach Nav Node** (`approach_nav_node.py`) — handles coarse navigation to a detected station once the perception stack identifies one

All navigation is coordinated by the **Supervisor Node** (`supervisor_node.py`) finite state machine, which decides when to explore, roam, or approach a target.

---

## From Custom Navigation to Nav2

Early in development, the team attempted to implement a custom navigation stack consisting of frontier detection, A* planning, obstacle inflation, and reactive control. The goal was to retain full control over path planning and obstacle avoidance without relying on a large external framework. However, this approach quickly proved unreliable and difficult to maintain.

Navigation behaviour was **inconsistent and highly sensitive to parameter tuning**, where small changes could lead to oscillations, stalled motion, or failure to reach goals. Debugging was also challenging due to the **lack of structured tools**, making it difficult to isolate whether issues stemmed from costmaps, planning, or control logic.

In addition, several practical challenges emerged:
- **Path planning** required extensive tuning and edge-case handling  
- **Recovery behaviours** had to be implemented from scratch  
- **Integration with SLAM** (via `slam_toolbox`) required careful TF management  

Given these limitations, the team transitioned to **Nav2**, which provides these components in a well-integrated and tested framework. This significantly improved reliability and reduced debugging effort, allowing the team to focus on higher-level mission logic such as frontier exploration, docking, and launching. Nav2 also integrates seamlessly with the `NavigateToPose` action interface, simplifying goal execution across system components.

---

## Nav2 Architecture

Nav2 is a modular navigation framework. The key servers used in this project are:

### Planner Server
Responsible for computing a global path from the robot's current position to the goal.

- **Plugin used:** `nav2_smac_planner/SmacPlanner2D`
- SmacPlanner2D uses an A*-based search on a 2D costmap grid, producing paths that respect costmap costs (unlike NavFn which treats all free cells equally)
- `cost_travel_multiplier: 3.0` and `cost_penalty: 2.0` bias the planner to prefer paths that stay away from high-cost (inflated) regions, resulting in paths that hug the centre of corridors rather than cutting close to walls
- `allow_unknown: false` — the planner will not route through unexplored cells; combined with `track_unknown_space: true` on the global costmap, unknown space is treated as an obstacle for planning purposes. This prevents the planner from sending the robot into unmapped areas during approach and docking phases
- `tolerance: 0.25` allows the planner to succeed if the exact goal cell is slightly unreachable
- `max_planning_time: 3.0 s` caps planning time to avoid blocking the system on complex environments

### Controller Server
Responsible for executing the global path using local trajectory planning.

- **Plugin used:** `nav2_mppi_controller::MPPIController` (Model Predictive Path Integral)
- MPPI samples a large batch of random trajectory rollouts, weights them by cost, and computes an optimal control update. It is well-suited to smooth, continuous environments and handles costmap-aware obstacle avoidance naturally
- `batch_size: 5000` — number of trajectory samples per iteration. Higher values improve solution quality at the cost of CPU load
- `time_steps: 56`, `model_dt: 0.05` — each trajectory is rolled out 56 steps × 50 ms = 2.8 seconds ahead
- `motion_model: "DiffDrive"` — correct model for the TurtleBot3's differential drive kinematics

Velocity limits:

```yaml
vx_max: 0.3       # m/s forward
vx_min: -0.05     # m/s reverse (minimal — prefer forward motion)
wz_max: 1.2       # rad/s rotation
```

The MPPI critics and their tuned weights:

| Critic | Weight | Purpose |
|---|---|---|
| `CostCritic` | 10.0 | Heavily penalises trajectories through high-cost (inflated) areas — primary obstacle avoidance |
| `PathAlignCritic` | 6.0 | Keeps the robot aligned with the global path |
| `PathFollowCritic` | 5.0 | Drives the robot to follow the path waypoints |
| `ConstraintCritic` | 4.0 | Penalises trajectories that violate velocity/acceleration limits |
| `GoalCritic` | 3.0 | Pulls the robot toward the goal when within 0.2 m |
| `PathAngleCritic` | 3.0 | Penalises large heading deviations from the path direction |
| `PreferForwardCritic` | 1.0 | Mild bias toward forward motion over reversing |
| `GoalAngleCritic` | disabled | Not needed — yaw at goal is not constrained |

`CostCritic.cost_weight: 10.0` was increased from the default to make obstacle avoidance more aggressive, which was necessary to prevent the robot from clipping inflated obstacle regions in narrow corridors.

`yaw_goal_tolerance: 6.28` (full circle) was set on the goal checker so the robot does not waste time rotating to a precise heading after reaching a frontier goal. Heading correction is handled separately during the docking phase.

### Behavior Server
Handles recovery behaviours when the robot gets stuck or a plan fails. Plugins enabled:

- `Spin` — rotates in place to clear sensor readings
- `BackUp` — reverses a short distance
- `DriveOnHeading` — drives forward a short distance
- `Wait` — pauses briefly before retrying

These run automatically via the BehaviorTree inside `bt_navigator` when the controller or planner reports failure.

### Smoother Server
After the global path is computed, the `SimpleSmoother` plugin smooths it to reduce sharp corners, making the controller's job easier.

### Costmaps

Two costmaps are maintained simultaneously:

**Global Costmap** (used by the planner):
- Frame: `map`
- Layers: `static_layer` (SLAM map), `obstacle_layer` (live LiDAR), `inflation_layer`
- `inflation_radius: 0.15 m`, `cost_scaling_factor: 4.0`
- `track_unknown_space: true` — unknown cells are tracked but `allow_unknown: false` on the planner means they are treated as obstacles for path planning
- Custom rectangular footprint `[[0.13, 0.11], [0.13, -0.14], [-0.20, -0.14], [-0.20, 0.11]]` used instead of a circular radius to accurately represent the robot's actual shape including the launcher attachment

**Local Costmap** (used by the controller):
- Frame: `odom`, rolling window 3×3 m around the robot
- Layers: `obstacle_layer`, `voxel_layer` (3D obstacle detection from LiDAR), `inflation_layer`
- `inflation_radius: 0.15 m`, `cost_scaling_factor: 4.0` — same as global for consistent cost scoring between planner and controller
- Same custom footprint as global costmap
- `footprint_padding: 0.04 m` adds a small safety buffer around the footprint

### SLAM (Simultaneous Localization and Mapping)

The system uses SLAM during exploration to both construct a map of the environment and estimate the robot's pose within it. Unlike AMCL, which assumes a pre-existing static map, SLAM operates in unknown environments by continuously updating an occupancy grid map while simultaneously performing localization.

The SLAM node (e.g. `slam_toolbox` in online asynchronous mode) publishes:
- `/map` (`OccupancyGrid`) — continuously updated map used by Nav2 costmaps and frontier detection
- `map` → `odom` transform — estimates robot pose relative to the evolving map

Nav2 uses this transform to perform global planning and localization without requiring a prebuilt map.

---

## Frontier Exploration (`frontier_node.py`)

### Concept

Frontier exploration is a classical autonomous exploration strategy. A **frontier** is defined as a free cell in the occupancy grid that is adjacent to at least one unknown cell. By navigating to frontiers, the robot progressively uncovers the map until no frontiers remain.

### Implementation

The `FrontierExplorer` node runs a 1-second timer loop (`explore_once`) that:

1. Gets the robot's current pose from TF (`map` → `base_link`, see [Appendix A](#appendix-a-tf-localization-lookup))
2. Reads the latest occupancy grid from `slam_toolbox`
3. Detects all frontier clusters via BFS from the robot's current cell
4. Scores clusters by information gain and selects the best target
5. Sends a `NavigateToPose` goal to Nav2

**Frontier detection** works in two passes (see [Appendix B](#appendix-b-bfs-reachability-algorithm-two-pass-frontier-detection)):
- A BFS from the robot's grid cell collects all reachable free cells
- Any free cell with **at least 2 unknown 8-connected neighbours** is marked as a frontier cell (stricter than the classic 1-neighbour definition — reduces noise from LiDAR speckle and map artifacts, see [Appendix C](#appendix-c-frontier-detection-code))
- Frontier cells are then clustered using 8-connectivity BFS into groups
- Only clusters with **≥8 cells** are kept (`frontier_min_size = 8`), filtering out tiny noise clusters

**Target selection** uses an **information gain heuristic** (see [Appendix D](#appendix-d-information-gain-scoring)):
- Each cluster is scored by `gain / distance^0.7`, where gain = cluster size (number of frontier cells)
- This balances exploration efficiency (prefer large unexplored regions) with proximity (don't ignore nearby frontiers)
- The cluster centroid is checked for obstacle clearance (3-cell wall clearance by default, relaxed to 2 then 0 if no valid targets found)
- If the centroid is not free, the closest free cell in the cluster is used instead

**Blacklisting:** Goals that are rejected by Nav2 or time out (45-second watchdog for EXPLORE, 67.5s for ROAM) are added to a `failed_goals` set and skipped in future iterations. This prevents the robot from repeatedly attempting unreachable frontiers.

**Empty frontier handling:** If no valid frontiers are detected, the node waits 3 seconds (to allow the SLAM map to update after arriving at a new position) and retries. After 3 consecutive empty checks, it declares frontiers exhausted and switches to ROAM mode (see [Appendix E](#appendix-e-empty-frontier-retry-logic)).

### Roam Mode

When all frontiers are exhausted but the mission is not yet complete (i.e. not all stations have been serviced), the supervisor transitions to `ROAM` mode. In this mode, the frontier node switches from structured frontier exploration to **reachability-based random navigation**:

**Target selection in ROAM** (see [Appendix F](#appendix-f-roam-target-selection)):
- Uses the same BFS reachability check as frontier detection to find all free cells the robot can actually reach
- Filters candidates to cells at least 30 cells (~1.5m) away from the robot
- Excludes cells within 10 cells (~0.5m) of previously visited roam goals (tracked in `roam_visited` set)
- Excludes cells within 2 cells of any obstacle
- Excludes previously failed goals
- If no candidates meet these criteria, relaxes distance requirement to 10 cells
- Picks randomly from the **farthest 20%** of candidates to maximize coverage

**Spin-to-scan behavior:**
After reaching each roam goal (or getting within 0.5m of it), the robot performs a **360° spin** using the Nav2 `Spin` action (see [Appendix G](#appendix-g-360-spin-after-roam-goal)). This gives the perception stack (`marker_mapper_node`) a full panoramic view to detect any station markers that may have been missed during initial exploration.

**Bidirectional transition:**
If new frontiers appear while roaming (e.g. after the SLAM map updates or the robot moves to a new vantage point), the frontier node **automatically switches back to EXPLORE mode**. This allows the robot to resume structured exploration if new areas become accessible.

This acts as a robust fallback to handle cases where a station was partially visible during exploration but not fully detected, or where the map topology changes after completing a mission.

### Topics

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/map` | Subscribe | `OccupancyGrid` | SLAM map for frontier detection |
| `/scan` | Subscribe | `LaserScan` | LiDAR data |
| `/robot_mode` | Subscribe | `String` | FSM mode from supervisor |
| `/frontiers_exhausted` | Publish | `Bool` | Signal when no frontiers remain |
| `/frontier_goal_marker` | Publish | `Marker` | RViz visualisation of current goal |

---

## Approach Navigation (`approach_nav_node.py`)

Once the perception stack (`marker_mapper_node`) detects a station and computes a coarse pre-docking pose, the supervisor transitions to `APPROACH_STATIONARY` or `APPROACH_DYNAMIC`. The `ApproachNavNode` then takes over navigation.

It subscribes to `/coarse_dock_goal` (`PoseStamped`) published by the marker mapper, and sends it directly to Nav2 via `NavigateToPose`. When Nav2 reports success, it publishes `/approach_done = True`, which triggers the supervisor to transition into the docking phase.

The approach node is intentionally simple — it delegates all path planning and obstacle avoidance to Nav2, and only manages the handoff between the FSM states.

### Topics

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/robot_mode` | Subscribe | `String` | FSM mode from supervisor |
| `/coarse_dock_goal` | Subscribe | `PoseStamped` | Target pose from perception |
| `/approach_done` | Publish | `Bool` | Signal approach completion to supervisor |

---

## Nav2 Tuning Notes

Tuning Nav2 for the TurtleBot3 in this project involved several iterations. Key decisions:

**Starting with DWB, switching to MPPI:**
The initial controller was `dwb_core::DWBLocalPlanner`. DWB is lightweight and its critic weights are easy to reason about, which made it a good starting point. However, DWB struggled in narrow corridors — it would oscillate or get stuck when the sampled velocity window didn't produce trajectories that could thread through tight spaces. MPPI was switched in because its large batch sampling (`batch_size: 5000`) explores a much wider range of trajectories and the `CostCritic` naturally steers away from inflated obstacle regions without needing manual critic weight balancing. The DWB config is preserved as commented-out YAML in the params file for reference.

**Switching from NavFn to SmacPlanner2D:**
`NavfnPlanner` was the initial planner. While simple and fast, NavFn treats all free cells as equal cost and can produce paths that cut close to walls. `SmacPlanner2D` was switched in because it is cost-aware — the `cost_travel_multiplier` and `cost_penalty` parameters bias it to prefer paths through low-cost (open) space, producing safer paths that stay away from inflated obstacle regions. This worked better with the MPPI controller since the global path it was tracking was already well-clear of obstacles.

**Footprint vs. radius:**
The default TurtleBot3 config uses `robot_radius: 0.16 m`. This was replaced with a custom rectangular footprint to account for the launcher hardware mounted on the robot, which extends the physical footprint asymmetrically. Using the actual footprint shape gives more accurate collision checking.

**Inflation radius:**
Both costmaps use `inflation_radius: 0.15 m` and `cost_scaling_factor: 4.0`. This was reduced from earlier iterations (which used 0.25 m) after finding that larger inflation caused the planner to fail in narrower parts of the environment where corridors were only slightly wider than the robot.

**Speed limits:**
`vx_max: 0.3 m/s` in MPPI but capped to `0.15 m/s` by the velocity smoother. The velocity smoother acts as a final safety cap and smooths acceleration, preventing jerky starts. `wz_max: 1.2 rad/s` allows fast in-place rotation for frontier navigation.

**`yaw_goal_tolerance: 6.28`** (effectively disabled) on the goal checker was a deliberate choice for frontier goals. Since the robot immediately re-evaluates the next frontier after arriving, precise heading at the goal is irrelevant and forcing a rotation wastes time.

---

## Appendices

### Appendix A: TF Localization Lookup

```python
# Declared in __init__
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# Used in update_robot_pose()
try:
    now = rclpy.time.Time()
    trans = self.tf_buffer.lookup_transform(
        'map', 'base_link', now, timeout=Duration(seconds=0.5)
    )
    self.robot_x = trans.transform.translation.x
    self.robot_y = trans.transform.translation.y
    q = trans.transform.rotation
    _, _, self.robot_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    return True
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    return False
```

### Appendix B: BFS Reachability Algorithm (Two-Pass Frontier Detection)

```python
# PASS 1: BFS to find all reachable free cells from robot position
start = self.world_to_grid(self.robot_x, self.robot_y)
reachable_free = set([start])
q = deque([start])
frontier_cells = set()

while q:
    r, c = q.popleft()
    if self.cell_is_frontier(r, c):
        frontier_cells.add((r, c))
    for nr, nc in self.neighbors4(r, c):
        if (nr, nc) not in reachable_free and self.is_free(nr, nc):
            reachable_free.add((nr, nc))
            q.append((nr, nc))

# PASS 2: Cluster frontier cells using 8-connectivity BFS
clusters = []
unvisited = set(frontier_cells)
while unvisited:
    seed = unvisited.pop()
    cluster = [seed]
    fq = deque([seed])
    while fq:
        r, c = fq.popleft()
        for nr, nc in self.neighbors8(r, c):
            if (nr, nc) in unvisited:
                unvisited.remove((nr, nc))
                cluster.append((nr, nc))
                fq.append((nr, nc))
    if len(cluster) >= self.frontier_min_size:
        clusters.append(cluster)
```

### Appendix C: Frontier Detection Code

```python
def cell_is_frontier(self, row: int, col: int) -> bool:
    """A frontier cell is free and touches at least 2 unknown cells (reduces noise)."""
    if not self.is_free(row, col):
        return False
    return sum(1 for nr, nc in self.neighbors8(row, col) if self.is_unknown(nr, nc)) >= 2
```

### Appendix D: Information Gain Scoring

```python
# Score each frontier cluster by information gain / distance ratio
dist = max(1, abs(r_mean - rr) + abs(c_mean - rc))
gain = len(cluster)
score = gain / (dist ** 0.7)

if score > best_score:
    best_score = score
    best_target = (r_mean, c_mean)
```

### Appendix E: Empty Frontier Retry Logic

```python
if not clusters:
    self._empty_frontier_count += 1
    self.get_logger().warn(
        f"No frontiers detected ({self._empty_frontier_count}/{self.empty_frontier_retries}) "
        f"— waiting {self.frontier_wait_s}s for map to update..."
    )
    self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + self.frontier_wait_s

    if self._empty_frontier_count >= self.empty_frontier_retries:
        self.get_logger().info("Frontiers exhausted → switching to ROAM")
        self.exploration_done = True
        self.current_mode = 'ROAM'
        self.exhausted_pub.publish(Bool(data=True))
```

### Appendix F: Roam Target Selection

```python
# Filter reachable cells for roam targets
candidates = [
    (r, c) for r, c in reachable
    if abs(r - rr) + abs(c - rc) >= 30
    and (r, c) not in self.failed_goals
    and not any(
        self.is_occupied(r + dr, c + dc)
        for dr in range(-clearance, clearance + 1)
        for dc in range(-clearance, clearance + 1)
    )
    and not any(
        abs(r - vr) + abs(c - vc) < self.roam_exclusion_radius
        for vr, vc in self.roam_visited
    )
]

# Pick from the farthest 20% to maximize coverage
candidates.sort(key=lambda cell: abs(cell[0] - rr) + abs(cell[1] - rc), reverse=True)
top_n = max(1, len(candidates) // 5)
return candidates[np.random.randint(top_n)]
```

### Appendix G: 360° Spin After Roam Goal

```python
def _do_spin_360(self):
    """Trigger a 360° spin via Nav2 Spin action after reaching a roam goal."""
    if not self.spin_client.wait_for_server(timeout_sec=3.0):
        self.get_logger().warn("Spin action server not available — skipping spin.")
        self._navigating = False
        return

    spin_goal = Spin.Goal()
    spin_goal.target_yaw = 2 * math.pi  # 360 degrees

    self.get_logger().info("[ROAM] Spinning 360° to scan for markers.")
    self._spinning = True
    spin_future = self.spin_client.send_goal_async(spin_goal)
    spin_future.add_done_callback(self._spin_response_cb)
```

