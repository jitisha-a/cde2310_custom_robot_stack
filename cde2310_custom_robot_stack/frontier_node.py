#!/usr/bin/env python3

import math
from collections import deque
from typing import List, Tuple, Optional, Set
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import Buffer, TransformListener

GridCell = Tuple[int, int]  # (row, col)

# ----------------------------
# Quaternion → Euler conversion
# ----------------------------
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# ----------------------------
# Frontier Explorer Node
# ----------------------------
class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data
        )

        self.current_mode = 'EXPLORE' #added new
        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )

        # Nav2 Action Client
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        # Frontier goal marker publisher
        self.marker_pub = self.create_publisher(Marker, '/frontier_goal_marker', 10)

        # Frontiers exhausted signal
        self.exhausted_pub = self.create_publisher(Bool, '/frontiers_exhausted', 10)

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map state
        self.map_grid: Optional[np.ndarray] = None
        self.map_info = None
        self.map_received = False

        # Robot pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # LiDAR state
        self.scan = np.array([])

        # Frontier params
        self.frontier_min_size = 8  # was 3 — filters LiDAR speckle and map artifacts
        self.failed_goals: Set[GridCell] = set()

        # Roam: exclusion zone around previously visited roam goals (in grid cells)
        # 0.5m radius / 0.05m resolution = 10 cells
        self.roam_visited: Set[GridCell] = set()
        self.roam_exclusion_radius = 10  # cells (~0.5m)

        # How many consecutive empty checks before giving up and switching to ROAM
        self.empty_frontier_retries = 3
        self._empty_frontier_count = 0
        # How long to wait between empty-frontier retries (seconds)
        # Gives the map time to update after arriving at a new position
        self.frontier_wait_s = 3.0

        # Goal state
        self._goal_handle = None
        self._navigating = False
        self._pending_goal = (0.0, 0.0)
        self._spinning = False
        self._cooldown_until = 0.0  # epoch seconds, next goal not sent before this

        # Timer-driven exploration loop
        self.explore_timer = self.create_timer(1.0, self.explore_once)

        self.exploration_done = False

    # ----------------------------
    # Callbacks
    # ----------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        raw = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        # 0=free, 1=occupied, -1=unknown
        grid = np.full_like(raw, -1, dtype=np.int8)
        grid[raw == -1] = -1
        grid[(raw >= 0) & (raw <= 50)] = 0
        grid[raw > 50] = 1

        # NO inflation needed; Nav2 handles obstacles
        self.map_grid = grid
        self.map_received = True

    def scan_callback(self, msg: LaserScan):
        self.scan = np.array(msg.ranges, dtype=np.float32)
        self.scan[self.scan == 0.0] = np.nan

    def mode_callback(self, msg):
        self.current_mode = msg.data
        # re-enable timer if switching into roam after frontier exploration ended
        if self.current_mode == 'ROAM' and self.explore_timer.is_canceled():
            self.explore_timer.reset()

    # ----------------------------
    # Grid helpers
    # ----------------------------
    def in_bounds(self, row: int, col: int) -> bool:
        return self.map_grid is not None and 0 <= row < self.map_grid.shape[0] and 0 <= col < self.map_grid.shape[1]

    def is_free(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and self.map_grid[row, col] == 0

    def is_unknown(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and self.map_grid[row, col] == -1

    def is_occupied(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and self.map_grid[row, col] == 1

    def world_to_grid(self, x: float, y: float) -> Optional[GridCell]:
        if self.map_info is None:
            return None
        res = self.map_info.resolution
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        return (row, col) if self.in_bounds(row, col) else None

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        res = self.map_info.resolution
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        x = ox + (col + 0.5) * res
        y = oy + (row + 0.5) * res
        return x, y

    def neighbors4(self, row: int, col: int):
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = row + dr, col + dc
            if self.in_bounds(nr, nc):
                yield nr, nc

    def neighbors8(self, row: int, col: int):
        for dr in [-1,0,1]:
            for dc in [-1,0,1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = row + dr, col + dc
                if self.in_bounds(nr, nc):
                    yield nr, nc

    # ----------------------------
    # Frontier detection
    # ----------------------------
    def cell_is_frontier(self, row: int, col: int) -> bool:
        """A frontier cell is free and touches at least 2 unknown cells (reduces noise)."""
        if not self.is_free(row, col):
            return False
        return sum(1 for nr, nc in self.neighbors8(row, col) if self.is_unknown(nr, nc)) >= 2

    def detect_frontiers(self) -> List[List[GridCell]]:
        """Detect all reachable frontier clusters from robot pose."""
        if self.map_grid is None or self.robot_x is None:
            return []

        start = self.world_to_grid(self.robot_x, self.robot_y)
        if start is None:
            return []

        # BFS reachable free cells
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

        # Cluster frontier cells using 8-connectivity
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

        return clusters

    def choose_frontier_target(self, clusters: List[List[GridCell]]) -> Optional[GridCell]:
        """
        Score frontiers by information gain / distance ratio.
        Falls back to relaxed obstacle check if strict check yields no candidates.
        """
        if not clusters:
            return None
        robot_cell = self.world_to_grid(self.robot_x, self.robot_y)
        if robot_cell is None:
            return None
        rr, rc = robot_cell

        def score_clusters(clearance, wall_clearance=3):
            best_target = None
            best_score = -1.0
            for cluster in clusters:
                r_mean = int(sum(r for r, _ in cluster) / len(cluster))
                c_mean = int(sum(c for _, c in cluster) / len(cluster))

                if not self.is_free(r_mean, c_mean):
                    free_cells = [(r, c) for r, c in cluster if self.is_free(r, c)]
                    if not free_cells:
                        continue
                    r_mean, c_mean = min(
                        free_cells,
                        key=lambda cell: abs(cell[0] - rr) + abs(cell[1] - rc)
                    )

                # skip if too close to any occupied cell
                if any(
                    self.is_occupied(r_mean + dr, c_mean + dc)
                    for dr in range(-wall_clearance, wall_clearance + 1)
                    for dc in range(-wall_clearance, wall_clearance + 1)
                ):
                    continue

                if clearance > 0 and any(
                    self.is_occupied(r_mean + dr, c_mean + dc)
                    for dr in range(-clearance, clearance + 1)
                    for dc in range(-clearance, clearance + 1)
                ):
                    continue

                dist = max(1, abs(r_mean - rr) + abs(c_mean - rc))
                gain = len(cluster)
                score = gain / (dist ** 0.7)

                if score > best_score:
                    best_score = score
                    best_target = (r_mean, c_mean)
            return best_target

        # Try strict wall clearance first, then relax if nothing found
        result = score_clusters(clearance=1, wall_clearance=3)
        if result is None:
            result = score_clusters(clearance=0, wall_clearance=2)
        if result is None:
            result = score_clusters(clearance=0, wall_clearance=0)
        return result

    # ----------------------------
    # Robot pose update
    # ----------------------------
    def update_robot_pose(self) -> bool:
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

    # ----------------------------
    # Nav2 Goal
    # ----------------------------
    def publish_goal_marker(self, x: float, y: float):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def send_goal(self, x: float, y: float):
        """Send a nav goal asynchronously — non-blocking."""
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 server not available!")
            self._navigating = False
            return

        yaw = math.atan2(y - self.robot_y, x - self.robot_x)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._pending_goal = (x, y)
        self._pending_goal_mode = self.current_mode
        send_future = self.nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Goal rejected — skipping.")
            cell = self.world_to_grid(*self._pending_goal)
            if cell:
                self.failed_goals.add(cell)
            self._navigating = False
            return
        self.get_logger().info("Goal accepted, navigating...")
        # watchdog: roam goals get 1.5x the explore timeout
        watchdog_s = 45.0 if self._pending_goal_mode != 'ROAM' else 45.0 * 1.5
        self.get_logger().info(f"Watchdog set to {watchdog_s:.1f}s (mode: {self._pending_goal_mode})")
        self._watchdog_timer = self.create_timer(watchdog_s, self._watchdog_cb)
        self._goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _watchdog_cb(self):
        """Cancel current goal if robot appears stuck after acceptance."""
        self._watchdog_timer.cancel()
        if not self._navigating:
            return
        self.get_logger().warn("Watchdog: robot not moving after goal acceptance — cancelling.")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        if self.current_mode == 'EXPLORE':
            cell = self.world_to_grid(*self._pending_goal)
            if cell:
                self.failed_goals.add(cell)
        else:
            self.get_logger().info("Watchdog cancel due to mode switch — not blacklisting.")
        self._navigating = False

    def _goal_result_cb(self, future):
        if hasattr(self, '_watchdog_timer'):
            self._watchdog_timer.cancel()

        result = future.result()
        self._goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully.")
            if self.current_mode == 'ROAM':
                # mark this area as visited so we don't return to it
                cell = self.world_to_grid(*self._pending_goal)
                if cell:
                    self.roam_visited.add(cell)
                self._do_spin_360()
                return  # _navigating cleared after spin completes
        else:
            if self.current_mode == 'ROAM':
                # in ROAM, if we got close enough just spin anyway
                cell = self.world_to_grid(*self._pending_goal)
                wx, wy = self._pending_goal
                dist = math.hypot(wx - self.robot_x, wy - self.robot_y) if self.robot_x is not None else 999
                if dist < 0.5:
                    self.get_logger().info(f"[ROAM] Close enough ({dist:.2f}m) — spinning anyway.")
                    self._do_spin_360()
                    return
                self.get_logger().warn(f"[ROAM] Goal failed (status {result.status}) — blacklisting.")
                if cell:
                    self.failed_goals.add(cell)
            elif self.current_mode == 'EXPLORE':
                self.get_logger().warn(
                    f"Goal failed (status {result.status}) — blacklisting."
                )
                cell = self.world_to_grid(*self._pending_goal)
                if cell:
                    self.failed_goals.add(cell)
            else:
                self.get_logger().info(
                    "Goal interrupted by mode switch (e.g. ArUco). Not marking failed."
                )

        self._navigating = False
        self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + 2.0  # 2s cooldown

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

    def _spin_response_cb(self, future):
        spin_handle = future.result()
        if not spin_handle.accepted:
            self.get_logger().warn("Spin goal rejected.")
            self._spinning = False
            self._navigating = False
            return
        spin_handle.get_result_async().add_done_callback(self._spin_result_cb)

    def _spin_result_cb(self, future):
        self._spinning = False
        self._navigating = False
        self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + 2.0
        self.get_logger().info("[ROAM] Spin complete. Ready for next roam goal.")

    # ----------------------------
    # Timer-driven exploration
    # ----------------------------
    def explore_once(self):

        if not self.map_received:
            return
        if self._navigating:
            return
        if self._spinning:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self._cooldown_until:
            return
        if not self.update_robot_pose():
            return

        mode = self.current_mode  

        # 2. ROAM MODE — navigate to a random free cell, but check for new frontiers first
        if mode == 'ROAM':
            # Check if new frontiers have appeared AND are actually navigable
            clusters = self.detect_frontiers()
            clusters = [
                c for c in clusters
                if not any(cell in self.failed_goals for cell in c)
            ]
            if clusters:
                # Only switch back if we can actually find a valid target
                target_cell = self.choose_frontier_target(clusters)
                if target_cell is not None:
                    self.get_logger().info('New frontiers found while roaming — switching back to EXPLORE.')
                    self.exploration_done = False
                    self.roam_visited.clear()
                    self.current_mode = 'EXPLORE'
                    self._empty_frontier_count = 0
                    self.exhausted_pub.publish(Bool(data=False))
                    return

            target = self.get_roam_target()
            if target is None:
                return

            wx, wy = self.grid_to_world(*target)
            self.get_logger().info(f"[ROAM] → ({wx:.2f}, {wy:.2f})")

            self.publish_goal_marker(wx, wy)
            self._navigating = True
            self.send_goal(wx, wy)
            return   # IMPORTANT: hard exit

        # 3. EXPLORE MODE
        if mode != 'EXPLORE':
            return

        clusters = self.detect_frontiers()
        clusters = [
            c for c in clusters
            if not any(cell in self.failed_goals for cell in c)
        ]

        if not clusters:
            self._empty_frontier_count += 1
            self.get_logger().warn(
                f"No frontiers detected ({self._empty_frontier_count}/{self.empty_frontier_retries}) "
                f"— waiting {self.frontier_wait_s}s for map to update..."
            )
            # Set cooldown so we wait before checking again — gives map time to update
            self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + self.frontier_wait_s

            if self._empty_frontier_count >= self.empty_frontier_retries:
                self.get_logger().info("Frontiers exhausted → switching to ROAM")
                self.exploration_done = True
                self.current_mode = 'ROAM'
                self.exhausted_pub.publish(Bool(data=True))
                self._goal_handle = None
                self._empty_frontier_count = 0
            return

        target_cell = self.choose_frontier_target(clusters)
        self._empty_frontier_count = 0  # reset — we found frontiers

        if target_cell is None:
            self.get_logger().warn('Frontiers exist but no valid target found — retrying next cycle.')
            self._cooldown_until = self.get_clock().now().nanoseconds / 1e9 + self.frontier_wait_s
            return

        wx, wy = self.grid_to_world(*target_cell)
        self.get_logger().info(f"[EXPLORE] → ({wx:.2f}, {wy:.2f})")

        self.publish_goal_marker(wx, wy)
        self._navigating = True
        self.send_goal(wx, wy)
    
    def get_roam_target(self):
        """Pick a reachable free cell far from robot using the same BFS as frontier detection."""
        if self.map_grid is None or self.robot_x is None:
            return None

        start = self.world_to_grid(self.robot_x, self.robot_y)
        if start is None:
            return None

        rr, rc = start
        clearance = 2

        # BFS to find all reachable free cells (same as detect_frontiers)
        reachable = set([start])
        q = deque([start])
        while q:
            r, c = q.popleft()
            for nr, nc in self.neighbors4(r, c):
                if (nr, nc) not in reachable and self.is_free(nr, nc):
                    reachable.add((nr, nc))
                    q.append((nr, nc))

        # filter: far enough, clear of obstacles, not previously failed, not near a visited roam goal
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

        if not candidates:
            # fallback: any reachable free cell at least 10 cells away, still avoiding visited
            candidates = [
                (r, c) for r, c in reachable
                if abs(r - rr) + abs(c - rc) >= 10
                and (r, c) not in self.failed_goals
                and not any(
                    abs(r - vr) + abs(c - vc) < self.roam_exclusion_radius
                    for vr, vc in self.roam_visited
                )
            ]

        if not candidates:
            return None

        # pick from the farthest 20% to maximise coverage
        candidates.sort(key=lambda cell: abs(cell[0] - rr) + abs(cell[1] - rc), reverse=True)
        top_n = max(1, len(candidates) // 5)
        return candidates[np.random.randint(top_n)]

# ----------------------------
# Main entry
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
