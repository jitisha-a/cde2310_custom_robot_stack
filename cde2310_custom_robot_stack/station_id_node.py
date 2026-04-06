#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32


class StationIdNode(Node):
    def __init__(self):
        super().__init__('station_id_node')

        self.current_mode = 'EXPLORE'
        self.role_marker_ids = [27, 29]

        # tuning
        self.rotate_speed = 0.2
        self.stationary_confirm_visible_s = 12.0
        self.dynamic_missing_thresh_s = 5.0

        self.active_role_marker_id = None
        self.last_seen_time = None
        self.first_seen_time = None
        self.done = False
        self.rotating = False

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.station_type_pub = self.create_publisher(String, '/station_type', 10)
        self.station_id_done_pub = self.create_publisher(Bool, '/station_id_done', 10)
        self.role_marker_pub = self.create_publisher(Int32, '/station_role_marker_id', 10)

        self.get_logger().info('Station ID node started.')

    def reset_state(self):
        self.active_role_marker_id = None
        self.last_seen_time = None
        self.first_seen_time = None
        self.done = False
        self.rotating = False
        self.stop_motion()

    def mode_callback(self, msg):
        old_mode = self.current_mode
        self.current_mode = msg.data

        if old_mode != 'STATION_ID' and self.current_mode == 'STATION_ID':
            self.get_logger().info('Entering STATION_ID mode. Resetting classifier.')
            self.reset_state()

        if self.current_mode != 'STATION_ID':
            self.stop_motion()

    def decode_compressed(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def detect_role_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params
        )

        if ids is None or len(ids) == 0:
            return []

        ids_flat = ids.flatten().tolist()
        return [mid for mid in ids_flat if mid in self.role_marker_ids]

    def publish_rotation(self):
        twist = Twist()
        twist.angular.z = self.rotate_speed
        self.cmd_pub.publish(twist)

    def stop_motion(self):
        self.cmd_pub.publish(Twist())

    def classify_station(self, station_type):
        if self.done:
            return

        self.done = True
        self.stop_motion()

        self.station_type_pub.publish(String(data=station_type))
        self.station_id_done_pub.publish(Bool(data=True))

        if self.active_role_marker_id is not None:
            self.role_marker_pub.publish(Int32(data=self.active_role_marker_id))

        self.get_logger().info(
            f'Station classified as {station_type.upper()} using role marker {self.active_role_marker_id}'
        )

    def image_callback(self, msg):
        if self.current_mode != 'STATION_ID' or self.done:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        frame = self.decode_compressed(msg)
        if frame is None:
            return

        visible_role_markers = self.detect_role_markers(frame)

        if not visible_role_markers:
            # if none visible, rotate to search
            self.publish_rotation()
            self.rotating = True

            # if we had already been seeing one and now lost it for long enough -> dynamic
            if self.active_role_marker_id is not None and self.last_seen_time is not None:
                missing_for = now - self.last_seen_time
                if missing_for >= self.dynamic_missing_thresh_s:
                    self.classify_station('dynamic')
            return

        # at least one visible
        self.stop_motion()
        self.rotating = False

        if self.active_role_marker_id is None:
            self.active_role_marker_id = visible_role_markers[0]
            self.first_seen_time = now
            self.get_logger().info(f'Role marker {self.active_role_marker_id} first seen.')

        elif self.active_role_marker_id not in visible_role_markers:
            # switch only if current one vanished and another appears
            self.active_role_marker_id = visible_role_markers[0]
            self.first_seen_time = now
            self.get_logger().info(f'Switched active role marker to {self.active_role_marker_id}.')

        self.last_seen_time = now

        # if continuously visible long enough -> stationary
        if self.first_seen_time is not None:
            visible_duration = now - self.first_seen_time
            if visible_duration >= self.stationary_confirm_visible_s:
                self.classify_station('stationary')


def main(args=None):
    rclpy.init(args=args)
    node = StationIdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
