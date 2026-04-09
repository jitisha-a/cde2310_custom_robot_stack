#!/usr/bin/env python3

import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage

import RPi.GPIO as GPIO


class DynamicLauncherHwNode(Node):
    def __init__(self):
        super().__init__('dynamic_launcher_hw_node')

        self.is_launching = False
        self.abort_requested = False
        self.launch_thread = None

        self.latest_frame = None
        self.dynamic_marker_id = 29

        # GPIO pins
        self.SERVO_PIN = 12
        self.ENA = 18
        self.IN1 = 23
        self.IN2 = 24

        self.MOTOR_PWM_FREQ = 1000
        self.SERVO_PWM_FREQ = 50
        self.MOTOR_DUTY = 30

        self.SERVO_HOLD_ANGLE = 120
        self.SERVO_LAUNCH_ANGLE = 180
        self.SERVO_MOVE_TIME = 0.5
        self.SERVO_RETURN_SETTLE = 0.2

        # image visibility settings
        self.full_frame_margin_px = 5

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        # ROS
        self.launch_cmd_sub = self.create_subscription(
            Bool, '/launch_dynamic_cmd', self.launch_cmd_callback, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10
        )
        self.launch_done_pub = self.create_publisher(Bool, '/launch_done', 10)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)

        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, self.SERVO_PWM_FREQ)
        self.motor_pwm = GPIO.PWM(self.ENA, self.MOTOR_PWM_FREQ)

        self.servo_pwm.start(0)
        self.motor_pwm.start(0)

        self.motor_stop()
        self.set_servo_angle(self.SERVO_HOLD_ANGLE)

        self.get_logger().info('Dynamic launcher HW node started on Pi.')

    # -------------------------
    # ROS callbacks
    # -------------------------
    def launch_cmd_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.is_launching:
            self.get_logger().warn('Dynamic launch already in progress.')
            return

        self.abort_requested = False
        self.launch_thread = threading.Thread(target=self.run_launch_sequence, daemon=True)
        self.launch_thread.start()

    def image_callback(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        self.latest_frame = frame

    # -------------------------
    # Motor helpers
    # -------------------------
    def motor_reverse(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)

    def motor_stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        self.motor_pwm.ChangeDutyCycle(0)

    def set_motor_speed(self, duty):
        duty = max(0, min(100, duty))
        self.motor_pwm.ChangeDutyCycle(duty)

    # -------------------------
    # Servo helper
    # -------------------------
    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        duty = 2 + (angle / 18.0)

        GPIO.output(self.SERVO_PIN, True)
        self.servo_pwm.ChangeDutyCycle(duty)
        time.sleep(self.SERVO_MOVE_TIME)
        GPIO.output(self.SERVO_PIN, False)
        self.servo_pwm.ChangeDutyCycle(0)

    # -------------------------
    # Utility
    # -------------------------
    def sleep_with_abort(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            if self.abort_requested:
                return False
            time.sleep(0.05)
        return True

    def marker_29_fully_visible(self):
        if self.latest_frame is None:
            return False

        frame = self.latest_frame
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

        # full in frame means all corners are inside image with a small margin
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

    def wait_for_marker_29_visible(self):
        self.get_logger().info('Waiting for marker 29 to appear fully in frame...')
        while rclpy.ok() and not self.abort_requested:
            if self.marker_29_fully_visible():
                self.get_logger().info('Marker 29 is fully visible.')
                return True
            time.sleep(0.05)
        return False

    def wait_for_marker_29_disappear(self):
        self.get_logger().info('Waiting for marker 29 to disappear from frame...')
        while rclpy.ok() and not self.abort_requested:
            if not self.marker_29_fully_visible():
                self.get_logger().info('Marker 29 disappeared.')
                return True
            time.sleep(0.05)
        return False

    def launch_one_ball(self, ball_number):
        self.get_logger().info(f'Dynamic launch: ball {ball_number}')
        self.set_servo_angle(self.SERVO_LAUNCH_ANGLE)
        self.set_servo_angle(self.SERVO_HOLD_ANGLE)

        if not self.sleep_with_abort(self.SERVO_RETURN_SETTLE):
            return False

        self.get_logger().info(f'Ball {ball_number} launched.')
        return True

    # -------------------------
    # Main sequence
    # -------------------------
    def run_launch_sequence(self):
        self.is_launching = True

        try:
            self.get_logger().info('Starting dynamic launch sequence.')

            # 1. Start motor immediately at 30% duty
            self.motor_reverse()
            self.set_motor_speed(self.MOTOR_DUTY)
            self.get_logger().info(f'Motor started immediately at {self.MOTOR_DUTY}% duty.')

            # 2. Ball 1: wait for visible
            if not self.wait_for_marker_29_visible():
                return
            if not self.launch_one_ball(1):
                return

            # 3. Ball 2: wait disappear, then visible
            if not self.wait_for_marker_29_disappear():
                return
            if not self.wait_for_marker_29_visible():
                return
            if not self.launch_one_ball(2):
                return

            # 4. Ball 3: wait disappear, then visible
            if not self.wait_for_marker_29_disappear():
                return
            if not self.wait_for_marker_29_visible():
                return
            if not self.launch_one_ball(3):
                return

            self.motor_stop()
            self.launch_done_pub.publish(Bool(data=True))
            self.get_logger().info('Published /launch_done = True')

        except Exception as e:
            self.get_logger().error(f'Error during dynamic launch sequence: {e}')

        finally:
            self.motor_stop()
            self.is_launching = False

    def destroy_node(self):
        try:
            self.abort_requested = True
            self.motor_pwm.ChangeDutyCycle(0)
            self.motor_stop()
            self.motor_pwm.stop()
            self.servo_pwm.stop()
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLauncherHwNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
