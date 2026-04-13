#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import RPi.GPIO as GPIO


class DynamicLauncherHwNode(Node):
    def __init__(self):
        super().__init__('dynamic_launcher_hw_node')

        self.is_launching = False
        self.abort_requested = False
        self.awaiting_launch = False

        self.ball_count = 0
        self.max_balls = 3

        # cooldown
        self.last_launch_time = 0.0
        self.launch_cooldown_sec = 5.0

        # GPIO pins
        self.SERVO_PIN = 12
        self.ENA = 18
        self.IN1 = 23
        self.IN2 = 24

        self.MOTOR_PWM_FREQ = 1000
        self.SERVO_PWM_FREQ = 50
        self.MOTOR_DUTY = 90

        self.SERVO_HOLD_ANGLE = 120
        self.SERVO_LAUNCH_ANGLE = 180
        self.SERVO_MOVE_TIME = 0.5
        self.SERVO_RETURN_SETTLE = 0.2

        # ROS
        self.launch_cmd_sub = self.create_subscription(
            Bool, '/launch_dynamic_cmd', self.launch_cmd_callback, 10
        )
        self.clearance_sub = self.create_subscription(
            Bool, '/dynamic_launch_clear', self.clearance_callback, 10
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

        self.start_launch_sequence()

    def clearance_callback(self, msg: Bool):
        if not msg.data:
            return

        if not self.is_launching:
            return

        if not self.awaiting_launch:
            return

        if self.ball_count >= self.max_balls:
            return

        # cooldown guard
        now = time.time()
        if now - self.last_launch_time < self.launch_cooldown_sec:
            self.get_logger().warn(
                f'Ignoring clearance: only {now - self.last_launch_time:.2f}s since last launch '
                f'(cooldown {self.launch_cooldown_sec:.1f}s).'
            )
            return

        self.awaiting_launch = False
        self.ball_count += 1

        self.get_logger().info(f'Received clearance for ball {self.ball_count}. Launching now.')
        success = self.launch_one_ball(self.ball_count)

        if not success:
            self.get_logger().warn('Launch aborted or failed during servo actuation.')
            self.finish_sequence(aborted=True)
            return

        self.last_launch_time = time.time()

        if self.ball_count >= self.max_balls:
            self.finish_sequence(aborted=False)
        else:
            self.awaiting_launch = True
            self.get_logger().info(
                f'Waiting for next clearance. Balls launched: {self.ball_count}/{self.max_balls}'
            )

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

    def sleep_with_abort(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            if self.abort_requested:
                return False
            time.sleep(0.05)
        return True

    # -------------------------
    # Launch control
    # -------------------------
    def start_launch_sequence(self):
        self.abort_requested = False
        self.is_launching = True
        self.awaiting_launch = True
        self.ball_count = 0
        self.last_launch_time = 0.0

        self.motor_reverse()
        self.set_motor_speed(self.MOTOR_DUTY)

        self.get_logger().info(
            f'Started dynamic launch sequence. Motor running reverse at {self.MOTOR_DUTY}% duty.'
        )
        self.get_logger().info('Waiting for first clearance signal from desktop.')

    def launch_one_ball(self, ball_number):
        self.get_logger().info(f'Launching ball {ball_number}...')
        self.set_servo_angle(self.SERVO_LAUNCH_ANGLE)
        self.set_servo_angle(self.SERVO_HOLD_ANGLE)

        if not self.sleep_with_abort(self.SERVO_RETURN_SETTLE):
            return False

        self.get_logger().info(f'Ball {ball_number} launched.')
        return True

    def finish_sequence(self, aborted=False):
        self.motor_stop()
        self.is_launching = False
        self.awaiting_launch = False

        if aborted:
            self.get_logger().warn('Dynamic launch sequence aborted.')
            return

        self.launch_done_pub.publish(Bool(data=True))
        self.get_logger().info('Dynamic launch sequence complete. Published /launch_done = True')

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
