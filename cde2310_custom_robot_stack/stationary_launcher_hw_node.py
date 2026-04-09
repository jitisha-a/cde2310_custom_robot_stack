#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import RPi.GPIO as GPIO


class StationaryLauncherHwNode(Node):
    def __init__(self):
        super().__init__('stationary_launcher_hw_node')

        self.has_launched = False
        self.is_launching = False
        self.abort_requested = False
        self.launch_thread = None

        self.SERVO_PIN = 12
        self.ENA = 18
        self.IN1 = 23
        self.IN2 = 24

        self.MOTOR_PWM_FREQ = 1000
        self.SERVO_PWM_FREQ = 50

        self.SERVO_HOLD_ANGLE = 120
        self.SERVO_LAUNCH_ANGLE = 170

        self.RAMP_STEP = 10
        self.RAMP_STEP_DELAY = 0.2
        self.MOTOR_SPINUP_WAIT = 3.0
        self.DELAY_AFTER_FIRST = 2.0
        self.DELAY_AFTER_SECOND = 8.0

        self.Max_duty = 90 # added new

        self.SERVO_MOVE_TIME = 0.5
        self.SERVO_RETURN_SETTLE = 0.2

        self.launch_cmd_sub = self.create_subscription(
            Bool, '/launch_stationary_cmd', self.launch_cmd_callback, 10
        )
        self.launch_done_pub = self.create_publisher(Bool, '/launch_done', 10)

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

        self.get_logger().info('Stationary launcher HW node started on Pi.')

    def launch_cmd_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.is_launching:
            self.get_logger().warn('Launch already in progress, ignoring new command.')
            return

        self.has_launched = False
        self.abort_requested = False
        self.launch_thread = threading.Thread(target=self.run_launch_sequence, daemon=True)
        self.launch_thread.start()

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

    def launch_one_ball(self, ball_number):
        self.get_logger().info(f'Launching ball {ball_number}...')
        self.set_servo_angle(self.SERVO_LAUNCH_ANGLE)
        self.set_servo_angle(self.SERVO_HOLD_ANGLE)

        if not self.sleep_with_abort(self.SERVO_RETURN_SETTLE):
            return False

        self.get_logger().info(f'Ball {ball_number} launched.')
        return True

    def run_launch_sequence(self):
        self.is_launching = True

        try:
            self.get_logger().info('Starting stationary launch sequence.')

            self.motor_reverse()
            speed = 0
            while speed < 30:
                if self.abort_requested:
                    return
                speed += self.RAMP_STEP
                speed = min(speed, self.Max_duty)
                self.set_motor_speed(speed)
                self.get_logger().info(f'Motor speed: {speed}%')
                if not self.sleep_with_abort(self.RAMP_STEP_DELAY):
                    return

            if not self.sleep_with_abort(self.MOTOR_SPINUP_WAIT):
                return

            if not self.launch_one_ball(1):
                return
            if not self.sleep_with_abort(self.DELAY_AFTER_FIRST):
                return
            if not self.launch_one_ball(2):
                return
            if not self.sleep_with_abort(self.DELAY_AFTER_SECOND):
                return
            if not self.launch_one_ball(3):
                return

            self.motor_stop()
            self.launch_done_pub.publish(Bool(data=True))
            self.has_launched = True
            self.get_logger().info('Published /launch_done = True')

        except Exception as e:
            self.get_logger().error(f'Error during stationary launch sequence: {e}')

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
    node = StationaryLauncherHwNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
