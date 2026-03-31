#!/usr/bin/env python3

import cv2
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import String


class MarkerWatcherNode(Node):
    def __init__(self):
        super().__init__('marker_watcher_node')

        self.target_id = 23
        self.required_hits = 5
        self.detection_window = deque(maxlen=self.required_hits)

        self.current_mode = 'EXPLORE'

        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        self.marker_pub = self.create_publisher(Bool, '/target_marker_found', 10)

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info('Marker watcher started.')

    def mode_callback(self, msg: String):
        self.current_mode = msg.data

    def image_callback(self, msg: CompressedImage):
        if self.current_mode != 'EXPLORE':
            return

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params
        )

        found = False
        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().tolist()
            found = self.target_id in ids_flat

        self.detection_window.append(found)

        msg_out = Bool()
        msg_out.data = len(self.detection_window) == self.required_hits and all(self.detection_window)
        self.marker_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerWatcherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
