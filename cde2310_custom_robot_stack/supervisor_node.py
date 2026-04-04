#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        # -------------------------
        # Publishers
        # -------------------------
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.launch_cmd_pub = self.create_publisher(Bool, '/launch_cmd', 10)

        # -------------------------
        # Subscribers
        # -------------------------
        self.coarse_goal_ready_sub = self.create_subscription(
            Bool,
            '/coarse_goal_ready',
            self.coarse_goal_ready_callback,
            10
        )

        self.approach_done_sub = self.create_subscription(
            Bool,
            '/approach_done',
            self.approach_done_callback,
            10
        )

        self.docking_done_sub = self.create_subscription(
            Bool,
            '/docking_done',
            self.docking_done_callback,
            10
        )

        self.launch_done_sub = self.create_subscription(
            Bool,
            '/launch_done',
            self.launch_done_callback,
            10
        )

        # Optional: if you later want to reset / cancel / fail
        # you can add more topics here.

        # -------------------------
        # Internal state
        # -------------------------
        self.current_mode = 'EXPLORE'
        self.last_published_mode = None

        # make sure /launch_cmd is only sent once per launch phase
        self.launch_command_sent = False

        # publish mode continuously / handle one-shot launch command
        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info('Supervisor started. Initial mode = EXPLORE')

    # =========================================================
    # Main periodic loop
    # =========================================================
    def loop(self):
        # Publish mode every cycle
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        # Log mode changes once
        if self.current_mode != self.last_published_mode:
            self.get_logger().info(f'Switching mode -> {self.current_mode}')
            self.last_published_mode = self.current_mode

        # If we've entered LAUNCH, send one launch command to Pi
        if self.current_mode == 'LAUNCH' and not self.launch_command_sent:
            self.get_logger().info('Publishing /launch_cmd = True')
            self.launch_cmd_pub.publish(Bool(data=True))
            self.launch_command_sent = True

    # =========================================================
    # Callbacks for state transitions
    # =========================================================
    def coarse_goal_ready_callback(self, msg: Bool):
        """
        EXPLORE -> APPROACH
        Marker mapper has computed a stable coarse docking goal in map frame.
        """
        if msg.data and self.current_mode == 'EXPLORE':
            self.get_logger().info(
                'Coarse docking goal ready. Switching from EXPLORE to APPROACH.'
            )
            self.current_mode = 'APPROACH'

    def approach_done_callback(self, msg: Bool):
        """
        APPROACH -> DOCK
        Nav2 has reached the coarse docking pose.
        """
        if msg.data and self.current_mode == 'APPROACH':
            self.get_logger().info(
                'Coarse approach complete. Switching from APPROACH to DOCK.'
            )
            self.current_mode = 'DOCK'

    def docking_done_callback(self, msg: Bool):
        """
        DOCK -> LAUNCH
        Visual docking has completed successfully.
        """
        if msg.data and self.current_mode == 'DOCK':
            self.get_logger().info(
                'Docking complete. Switching from DOCK to LAUNCH.'
            )
            self.current_mode = 'LAUNCH'
            self.launch_command_sent = False

    def launch_done_callback(self, msg: Bool):
        """
        LAUNCH -> IDLE
        Pi hardware launcher finished sequence and reported done.
        """
        if msg.data and self.current_mode == 'LAUNCH':
            self.get_logger().info(
                'Launch complete. Switching from LAUNCH to IDLE.'
            )
            self.current_mode = 'IDLE'
            self.launch_command_sent = False


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
