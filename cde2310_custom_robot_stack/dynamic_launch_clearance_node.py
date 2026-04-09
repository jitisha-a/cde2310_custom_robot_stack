#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool


class DynamicLaunchClearanceNode(Node):
    def __init__(self):
        super().__init__('dynamic_launch_clearance_node')

        self.current_mode = 'EXPLORE'
        self.dynamic_marker_id = 29

        # publish one clearance only when marker becomes newly visible again
        self.marker_currently_visible = False
        self.clearance_count = 0
        self.max_clearances = 3

        # full in frame margin
        self.full_frame_margin_px = 5

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10
        )

        self.clearance_pub = self.create_publisher(Bool, '/dynamic_launch_clear', 10)

        self.get_logger().info('Dynamic launch clearance node started.')

    def mode_callback(self, msg):
        old_mode = self.current_mode
        self.current_mode = msg.data

        if old_mode != 'LAUNCH_DYNAMIC' and self.current_mode == 'LAUNCH_DYNAMIC':
            self.get_logger().info('Entering LAUNCH_DYNAMIC. Resetting clearance state.')
            self.marker_currently_visible = False
            self.clearance_count = 0

        if self.current_mode != 'LAUNCH_DYNAMIC':
            self.marker_currently_visible = False

    def decode_compressed(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def marker_29_fully_visible(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params
        )

        if ids is None or len(ids) == 0:
            return False

        ids_flat = ids.flatten().tolist()
        if self.dynamic_marker_id not in ids_flat:
            return False

        idx = ids_flat.index(self.dynamic_marker_id)
        corners = corners_list[idx].reshape(4, 2)

        h, w = gray.shape[:2]

        for (x, y) in corners:
            if x < self.full_frame_margin_px:
                return False
            if x > (w - self.full_frame_margin_px):
                return False
            if y < self.full_frame_margin_px:
                return False
            if y > (h - self.full_frame_margin_px):
                return False

        return True

    def image_callback(self, msg):
        if self.current_mode != 'LAUNCH_DYNAMIC':
            return

        if self.clearance_count >= self.max_clearances:
            return

        frame = self.decode_compressed(msg)
        if frame is None:
            return

        fully_visible = self.marker_29_fully_visible(frame)

        # Rising edge: not visible before, visible now
        if fully_visible and not self.marker_currently_visible:
            self.marker_currently_visible = True
            self.clearance_count += 1
            self.clearance_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'Marker 29 fully visible. Published clearance {self.clearance_count}/{self.max_clearances}.'
            )

        # Falling edge: visible before, not visible now
        elif not fully_visible and self.marker_currently_visible:
            self.marker_currently_visible = False
            self.get_logger().info('Marker 29 left full view. Waiting for next reappearance.')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLaunchClearanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
