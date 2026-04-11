#!/usr/bin/env python3

import math
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Int32

import tf2_ros

from ament_index_python.packages import get_package_share_directory
import os


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


class MarkerMapperNode(Node):
    def __init__(self):
        super().__init__('marker_mapper_node')

        self.current_mode = 'EXPLORE'
        self.station_marker_ids = [23, 25]
        self.marker_length_m = 0.04
        self.required_samples = 5
        self.goal_history = deque(maxlen=self.required_samples)
        self.current_candidate_id = None
        self.standoff_distance_m = 0.35

        self.camera_x_offset_m = 0.05
        self.camera_y_offset_m = 0.15

        pkg_share = get_package_share_directory('cde2310_custom_robot_stack')
        self.calib_file = os.path.join(pkg_share, 'config', 'camera_calib_640x480.npz')

        data = np.load(self.calib_file)
        self.camMatrix = data["camMatrix"]
        self.distCoeffs = data["distCoeffs"]

        L = self.marker_length_m
        self.objPoints = np.array([
            [-L / 2.0,  L / 2.0, 0.0],
            [ L / 2.0,  L / 2.0, 0.0],
            [ L / 2.0, -L / 2.0, 0.0],
            [-L / 2.0, -L / 2.0, 0.0],
        ], dtype=np.float32)

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/coarse_dock_goal', 10)
        self.goal_ready_pub = self.create_publisher(Bool, '/coarse_goal_ready', 10)
        self.target_marker_pub = self.create_publisher(Int32, '/target_station_marker_id', 10)
        self.target_type_pub = self.create_publisher(String, '/target_station_type', 10)

        self.serviced_marker_ids = set()

        self.serviced_marker_sub = self.create_subscription(
            Int32, '/serviced_station_marker_id', self.serviced_marker_callback, 10
        )

        self.get_logger().info('Marker mapper node started.')

    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode not in ('EXPLORE', 'ROAM'):
            self.goal_history.clear()
            self.current_candidate_id = None

    def decode_compressed(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def detect_station_marker_pose(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params
        )

        if ids is None or len(ids) == 0:
            return False, None, None, None

        ids_flat = ids.flatten().tolist()

        chosen_id = None
        chosen_idx = None
        for sid in self.station_marker_ids:
            if sid in self.serviced_marker_ids:
                continue
            if sid in ids_flat:
                chosen_id = sid
                chosen_idx = ids_flat.index(sid)
                break

        if chosen_id is None:
            return False, None, None, None

        corners = corners_list[chosen_idx].reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            self.objPoints,
            corners,
            self.camMatrix,
            self.distCoeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not ok:
            return False, None, None, None

        return True, chosen_id, rvec, tvec

    def get_robot_pose_map(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            return x, y, yaw
        except Exception:
            return None

    def marker_id_to_station_type(self, marker_id):
        if marker_id == 23:
            return 'stationary'
        if marker_id == 25:
            return 'dynamic'
        return ''

    def serviced_marker_callback(self, msg: Int32):
        self.serviced_marker_ids.add(msg.data)
        self.get_logger().info(f'Marker {msg.data} permanently marked as serviced and will be ignored.')
    
    def image_callback(self, msg):
        if self.current_mode not in ('EXPLORE', 'ROAM'):
            return

        frame = self.decode_compressed(msg)
        if frame is None:
            return

        found, marker_id, rvec, tvec = self.detect_station_marker_pose(frame)
        if not found:
            return

        if self.current_candidate_id is None:
            self.current_candidate_id = marker_id

        if marker_id != self.current_candidate_id:
            self.goal_history.clear()
            self.current_candidate_id = marker_id

        robot_pose = self.get_robot_pose_map()
        if robot_pose is None:
            self.get_logger().warn('Could not get map->base_link transform yet.')
            return

        robot_x, robot_y, robot_yaw = robot_pose

        tx = float(tvec[0, 0])
        tz = float(tvec[2, 0])

        marker_local_x = self.camera_x_offset_m + tz
        marker_local_y = self.camera_y_offset_m - tx

        marker_map_x = robot_x + math.cos(robot_yaw) * marker_local_x - math.sin(robot_yaw) * marker_local_y
        marker_map_y = robot_y + math.sin(robot_yaw) * marker_local_x + math.cos(robot_yaw) * marker_local_y

        vec_x = marker_map_x - robot_x
        vec_y = marker_map_y - robot_y
        dist = math.hypot(vec_x, vec_y)
        if dist < 1e-6:
            return

        unit_x = vec_x / dist
        unit_y = vec_y / dist

        goal_x = marker_map_x - self.standoff_distance_m * unit_x
        goal_y = marker_map_y - self.standoff_distance_m * unit_y
        goal_yaw = math.atan2(marker_map_y - goal_y, marker_map_x - goal_x)

        self.goal_history.append((goal_x, goal_y, goal_yaw))

        if len(self.goal_history) < self.required_samples:
            return

        avg_x = float(np.mean([g[0] for g in self.goal_history]))
        avg_y = float(np.mean([g[1] for g in self.goal_history]))
        sin_avg = float(np.mean([math.sin(g[2]) for g in self.goal_history]))
        cos_avg = float(np.mean([math.cos(g[2]) for g in self.goal_history]))
        avg_yaw = math.atan2(sin_avg, cos_avg)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = avg_x
        goal_msg.pose.position.y = avg_y
        goal_msg.pose.orientation.z = math.sin(avg_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(avg_yaw / 2.0)

        station_type = self.marker_id_to_station_type(self.current_candidate_id)

        self.goal_pub.publish(goal_msg)
        self.goal_ready_pub.publish(Bool(data=True))
        self.target_marker_pub.publish(Int32(data=self.current_candidate_id))
        self.target_type_pub.publish(String(data=station_type))

        self.get_logger().info(
            f'Coarse goal ready for marker {self.current_candidate_id} ({station_type}): '
            f'x={avg_x:.2f}, y={avg_y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
