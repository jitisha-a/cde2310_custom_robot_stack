#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.launch_stationary_cmd_pub = self.create_publisher(Bool, '/launch_stationary_cmd', 10)
        self.launch_dynamic_cmd_pub = self.create_publisher(Bool, '/launch_dynamic_cmd', 10)
        self.serviced_marker_pub = self.create_publisher(Int32, '/serviced_station_marker_id', 10)

        self.target_station_marker_sub = self.create_subscription(
            Int32, '/target_station_marker_id', self.target_station_marker_callback, 10
        )
        
        self.target_station_marker_id = None

        self.coarse_goal_ready_sub = self.create_subscription(
            Bool, '/coarse_goal_ready', self.coarse_goal_ready_callback, 10
        )
        self.target_station_type_sub = self.create_subscription(
            String, '/target_station_type', self.target_station_type_callback, 10
        )
        self.approach_done_sub = self.create_subscription(
            Bool, '/approach_done', self.approach_done_callback, 10
        )
        self.docking_done_sub = self.create_subscription(
            Bool, '/docking_done', self.docking_done_callback, 10
        )
        self.launch_done_sub = self.create_subscription(
            Bool, '/launch_done', self.launch_done_callback, 10
        )

        self.current_mode = 'EXPLORE'
        self.last_published_mode = None
        self.target_station_type = ''

        self.stationary_launch_sent = False
        self.dynamic_launch_sent = False
        self.missions_completed = 0
        self.missions_required = 2  # one stationary + one dynamic

        self.frontiers_exhausted = False
        self.frontiers_sub = self.create_subscription(
            Bool, '/frontiers_exhausted', self.frontiers_exhausted_callback, 10
        )

        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info('Supervisor started. Initial mode = EXPLORE')

    def loop(self):
        self.mode_pub.publish(String(data=self.current_mode))

        if self.current_mode != self.last_published_mode:
            self.get_logger().info(f'Switching mode -> {self.current_mode}')
            self.last_published_mode = self.current_mode

        if self.current_mode == 'LAUNCH_STATIONARY' and not self.stationary_launch_sent:
            self.get_logger().info('Publishing /launch_stationary_cmd = True')
            self.launch_stationary_cmd_pub.publish(Bool(data=True))
            self.stationary_launch_sent = True

        if self.current_mode == 'LAUNCH_DYNAMIC' and not self.dynamic_launch_sent:
            self.get_logger().info('Publishing /launch_dynamic_cmd = True')
            self.launch_dynamic_cmd_pub.publish(Bool(data=True))
            self.dynamic_launch_sent = True

    # -------------------------
    # Callbacks
    # -------------------------
    def frontiers_exhausted_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'EXPLORE':
            self.frontiers_exhausted = True
            if self.missions_completed < self.missions_required:
                self.get_logger().info('Frontiers exhausted but missions incomplete. Switching to ROAM.')
                self.current_mode = 'ROAM'
            else:
                self.get_logger().info('Frontiers exhausted and all missions complete.')

    def coarse_goal_ready_callback(self, msg: Bool):
        if msg.data and self.current_mode in ('EXPLORE', 'ROAM'):
            if self.target_station_type == 'stationary':
                self.get_logger().info('Stationary target found. Switching to APPROACH_STATIONARY.')
                self.current_mode = 'APPROACH_STATIONARY'
            elif self.target_station_type == 'dynamic':
                self.get_logger().info('Dynamic target found. Switching to APPROACH_DYNAMIC.')
                self.current_mode = 'APPROACH_DYNAMIC'
            else:
                self.get_logger().warn('Coarse goal ready but target station type unknown.')

    def approach_done_callback(self, msg: Bool):
        if not msg.data:
            return
    
        if self.current_mode == 'APPROACH_STATIONARY':
            self.get_logger().info('Approach complete. Switching to DOCK_STATIONARY.')
            self.current_mode = 'DOCK_STATIONARY'

        elif self.current_mode == 'APPROACH_DYNAMIC':
            self.get_logger().info('Approach complete. Switching to DOCK_DYNAMIC.')
            self.current_mode = 'DOCK_DYNAMIC'

    def target_station_type_callback(self, msg: String):
        self.target_station_type = msg.data


    def docking_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.current_mode == 'DOCK_STATIONARY':
            self.get_logger().info('Docking complete. Switching to LAUNCH_STATIONARY.')
            self.current_mode = 'LAUNCH_STATIONARY'
            self.stationary_launch_sent = False

        elif self.current_mode == 'DOCK_DYNAMIC':
            self.get_logger().info('Docking complete. Switching to LAUNCH_DYNAMIC.')
            self.current_mode = 'LAUNCH_DYNAMIC'
            self.dynamic_launch_sent = False

    def target_station_marker_callback(self, msg: Int32):
        self.target_station_marker_id = msg.data

    def launch_done_callback(self, msg: Bool):
    if not msg.data:
        return

    if self.current_mode in ['LAUNCH_STATIONARY', 'LAUNCH_DYNAMIC']:
        if self.target_station_marker_id in [23, 25]:
            self.get_logger().info(
                f'Launch complete. Marking station marker {self.target_station_marker_id} as serviced.'
            )
            self.serviced_marker_pub.publish(Int32(data=self.target_station_marker_id))

        self.get_logger().info('Launch complete. Returning to EXPLORE.')
        self.current_mode = 'EXPLORE'
        self.target_station_type = ''
        self.stationary_launch_sent = False
        self.dynamic_launch_sent = False
        self.target_station_marker_id = None

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
