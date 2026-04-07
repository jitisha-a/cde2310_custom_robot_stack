#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool, Float32


class TimingGaugeNode(Node):
    def __init__(self):
        super().__init__('timing_gauge_node')

        self.current_mode = 'EXPLORE'
        self.done = False

        self.state = 'WAIT_FIRST_VISIBLE'
        self.x_start = None
        self.y_start = None
        self.measured_x = None
        self.measured_y = None

        self.dynamic_marker_id = 29

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10
        )

        self.x_pub = self.create_publisher(Float32, '/measured_x_sec', 10)
        self.y_pub = self.create_publisher(Float32, '/measured_y_sec', 10)
        self.gauge_done_pub = self.create_publisher(Bool, '/gauge_done', 10)

        self.get_logger().info('Timing gauge node started.')

    def reset_state(self):
        self.done = False
        self.state = 'WAIT_FIRST_VISIBLE'
        self.x_start = None
        self.y_start = None
        self.measured_x = None
        self.measured_y = None

    def mode_callback(self, msg):
        old_mode = self.current_mode
        self.current_mode = msg.data

        if old_mode != 'GAUGE_DYNAMIC' and self.current_mode == 'GAUGE_DYNAMIC':
            self.get_logger().info('Entering GAUGE_DYNAMIC mode. Resetting gauge state.')
            self.reset_state()

    def decode_compressed(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def detect_ids(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params
        )
        if ids is None or len(ids) == 0:
            return []
        return ids.flatten().tolist()

    def image_callback(self, msg):
        if self.current_mode != 'GAUGE_DYNAMIC' or self.done:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        frame = self.decode_compressed(msg)
        if frame is None:
            return

        ids = self.detect_ids(frame)
        visible = self.dynamic_marker_id in ids

        if self.state == 'WAIT_FIRST_VISIBLE':
            if visible:
                self.get_logger().info('First visible window seen. Skipping it.')
                self.state = 'WAIT_FIRST_DISAPPEAR'

        elif self.state == 'WAIT_FIRST_DISAPPEAR':
            if not visible:
                self.get_logger().info('First visible window ended. Waiting for second visible window.')
                self.state = 'WAIT_SECOND_VISIBLE'

        elif self.state == 'WAIT_SECOND_VISIBLE':
            if visible:
                self.x_start = now
                self.get_logger().info('Second visible window started. Measuring x.')
                self.state = 'MEASURE_X'

        elif self.state == 'MEASURE_X':
            if not visible:
                self.measured_x = now - self.x_start
                self.y_start = now
                self.get_logger().info(f'Measured x = {self.measured_x:.2f}s. Measuring hidden interval y.')
                self.state = 'MEASURE_Y'

        elif self.state == 'MEASURE_Y':
            if visible:
                self.measured_y = now - self.y_start
                period = 2 * self.measured_x + 2 * self.measured_y

                self.x_pub.publish(Float32(data=float(self.measured_x)))
                self.y_pub.publish(Float32(data=float(self.measured_y)))
                self.gauge_done_pub.publish(Bool(data=True))

                self.get_logger().info(
                    f'Measured y = {self.measured_y:.2f}s, estimated period = {period:.2f}s'
                )
                self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = TimingGaugeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
