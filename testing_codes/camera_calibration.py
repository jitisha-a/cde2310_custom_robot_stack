#!/usr/bin/env python3
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class HeadlessCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrate_headless')

        self.topic = '/image_raw/compressed'

        # Chessboard INNER corners
        self.board_cols = 9
        self.board_rows = 6
        self.board_size = (self.board_cols, self.board_rows)

        # Square size in meters
        self.square_size_m = 0.025

        # How many good captures to collect
        self.target_captures = 20

        # Cooldown between captures (seconds) so i can move board
        self.capture_cooldown_s = 0.6

        # Save output
        self.out_file = 'camera_calib_640x480.npz'
        # ------------------------------------------------------

        # Storage for calibration data
        self.objpoints = []
        self.imgpoints = []
        self.image_size = None  # (w, h)

        # Precompute 3D chessboard model points for one view
        objp = np.zeros((self.board_rows * self.board_cols, 3), dtype=np.float32)
        objp[:, :2] = np.mgrid[0:self.board_cols, 0:self.board_rows].T.reshape(-1, 2)
        objp *= self.square_size_m
        self.objp_single = objp

        # Corner refinement criteria
        self.subpix_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )

        # Capture timing
        self.last_capture_time = 0.0

        # ROS subscription
        self.sub = self.create_subscription(CompressedImage, self.topic, self.cb, 10)

        self.get_logger().info(
            "Headless camera calibration started.\n"
            f"Topic: {self.topic}\n"
            f"Chessboard inner corners: {self.board_cols}x{self.board_rows}\n"
            f"Square size: {self.square_size_m} m\n"
            f"Target captures: {self.target_captures}\n"
            f"Cooldown: {self.capture_cooldown_s} s\n"
            f"Output: {os.path.abspath(self.out_file)}\n"
            "Move the chessboard around (different angles/distances)."
        )

    def decode_compressed(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return frame

    def cb(self, msg: CompressedImage):
        # If done, ignore further frames
        if len(self.imgpoints) >= self.target_captures:
            return

        frame = self.decode_compressed(msg)
        if frame is None:
            self.get_logger().warn("Failed to decode compressed frame.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape[:2]
        self.image_size = (w, h)

        found, corners = cv2.findChessboardCorners(gray, self.board_size, None)
        if not found:
            # Keep quiet to avoid spamming logs
            return

        # Refine corners to subpixel accuracy
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), self.subpix_criteria
        )

        # Enforce cooldown so you can move the board between captures
        now = time.time()
        if now - self.last_capture_time < self.capture_cooldown_s:
            return

        # Capture this sample
        self.objpoints.append(self.objp_single.copy())
        self.imgpoints.append(corners_refined)
        self.last_capture_time = now

        self.get_logger().info(f"Captured {len(self.imgpoints)}/{self.target_captures}")

        # If we hit the target, calibrate immediately
        if len(self.imgpoints) >= self.target_captures:
            self.get_logger().info("Collected enough samples. Calibrating now...")
            self.calibrate_and_save()
            rclpy.shutdown()

    def calibrate_and_save(self):
        if self.image_size is None or len(self.imgpoints) < 12:
            self.get_logger().error("Not enough data to calibrate.")
            return

        ret, camMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            self.image_size,
            None,
            None
        )

        # Compute reprojection RMSE in pixels (helpful quality metric)
        total_err2 = 0.0
        total_points = 0
        for i in range(len(self.objpoints)):
            proj, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], camMatrix, distCoeffs)
            err = cv2.norm(self.imgpoints[i], proj, cv2.NORM_L2)
            total_err2 += err * err
            total_points += len(self.objpoints[i])

        rmse = np.sqrt(total_err2 / total_points) if total_points > 0 else float('nan')

        self.get_logger().info(f"OpenCV RMS (ret): {ret}")
        self.get_logger().info(f"Reprojection RMSE (px): {rmse:.4f}")
        self.get_logger().info(f"camMatrix:\n{camMatrix}")
        self.get_logger().info(f"distCoeffs:\n{distCoeffs.ravel()}")

        np.savez(
            self.out_file,
            camMatrix=camMatrix,
            distCoeffs=distCoeffs,
            image_size=np.array(self.image_size, dtype=np.int32)
        )
        self.get_logger().info(f"Saved calibration to: {os.path.abspath(self.out_file)}")


def main():
    rclpy.init()
    node = HeadlessCalibrator()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
