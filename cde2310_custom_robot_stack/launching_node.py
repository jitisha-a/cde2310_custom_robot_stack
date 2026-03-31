#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import RPi.GPIO as GPIO


class LaunchingNode(Node):
    def __init__(self):
        super().__init__('launching_node')

        # -------------------------
        # ROS state
        # -------------------------
        self.current_mode = 'IDLE'
        self.has_launched = False
        self.is_launching = False
        self.abort_requested = False
        self.launch_thread = None

        # -------------------------
        # GPIO pin config (BCM)
        # -------------------------
        self.SERVO_PIN = 12     # keep servo on BCM 12
        self.ENA = 18           # motor PWM pin, changed from 12 to avoid conflict
        self.IN1 = 23           # motor direction pin 1
        self.IN2 = 24           # motor direction pin 2

        self.MOTOR_PWM_FREQ = 1000   # 1 kHz
        self.SERVO_PWM_FREQ = 50     # standard servo PWM

        # -------------------------
        # Launch sequence settings
        # -------------------------
        self.SERVO_HOLD_ANGLE = 120
        self.SERVO_LAUNCH_ANGLE = 180

        self.RAMP_STEP = 10                 # percent per step
        self.RAMP_STEP_DELAY = 0.2          # seconds between steps
        self.MOTOR_SPINUP_WAIT = 3.0        # wait after reaching 100%
        self.DELAY_AFTER_FIRST = 2.0        # second launch after 2 sec
        self.DELAY_AFTER_SECOND = 8.0       # third launch after 8 sec

        self.SERVO_MOVE_TIME = 0.5          # time to let servo reach angle
        self.SERVO_RETURN_SETTLE = 0.2      # short settle after reset

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )

        self.launch_done_pub = self.create_publisher(Bool, '/launch_done', 10)

        # -------------------------
        # GPIO setup
        # -------------------------
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

        self.get_logger().info('Launching node started.')

    # =========================
    # ROS callbacks
    # =========================
    def mode_callback(self, msg: String):
        new_mode = msg.data

        if new_mode != self.current_mode:
            self.get_logger().info(f'Mode changed: {self.current_mode} -> {new_mode}')

        self.current_mode = new_mode

        # Reset one-shot flag when leaving LAUNCH
        if self.current_mode != 'LAUNCH':
            self.has_launched = False

            # Optional abort if mode changes away during launch
            if self.is_launching:
                self.get_logger().warn('Mode left LAUNCH during launch sequence. Abort requested.')
                self.abort_requested = True

        # Start launch once when entering LAUNCH
        if self.current_mode == 'LAUNCH' and not self.has_launched and not self.is_launching:
            self.abort_requested = False
            self.launch_thread = threading.Thread(target=self.run_launch_sequence, daemon=True)
            self.launch_thread.start()

    # =========================
    # Motor helpers
    # =========================
    def motor_forward(self):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)

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

    # =========================
    # Servo helper
    # =========================
    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        duty = 2 + (angle / 18.0)

        GPIO.output(self.SERVO_PIN, True)
        self.servo_pwm.ChangeDutyCycle(duty)
        time.sleep(self.SERVO_MOVE_TIME)
        GPIO.output(self.SERVO_PIN, False)
        self.servo_pwm.ChangeDutyCycle(0)

    # =========================
    # Timing helper
    # =========================
    def sleep_with_abort(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            if self.abort_requested:
                return False
            time.sleep(0.05)
        return True

    # =========================
    # Ball launch action
    # =========================
    def launch_one_ball(self, ball_number):
        self.get_logger().info(f'Launching ball {ball_number}...')

        self.set_servo_angle(self.SERVO_LAUNCH_ANGLE)
        self.set_servo_angle(self.SERVO_HOLD_ANGLE)

        if not self.sleep_with_abort(self.SERVO_RETURN_SETTLE):
            return False

        self.get_logger().info(f'Ball {ball_number} launched.')
        return True

    # =========================
    # Main launch sequence
    # =========================
    def run_launch_sequence(self):
        self.is_launching = True

        try:
            self.get_logger().info('LAUNCH mode active. Starting launcher sequence.')

            # 1. Power on motor in reverse direction
            self.motor_reverse()
            self.get_logger().info('Motor set to reverse.')

            # 2. Ramp motor gradually to 100%
            speed = 0
            while speed < 100:
                if self.abort_requested:
                    self.get_logger().warn('Launch aborted during motor ramp.')
                    return

                speed += self.RAMP_STEP
                speed = min(speed, 100)
                self.set_motor_speed(speed)
                self.get_logger().info(f'Motor speed: {speed}%')

                if not self.sleep_with_abort(self.RAMP_STEP_DELAY):
                    self.get_logger().warn('Launch aborted during motor ramp delay.')
                    return

            # 3. Wait before first launch
            self.get_logger().info(f'Waiting {self.MOTOR_SPINUP_WAIT:.1f}s before first launch...')
            if not self.sleep_with_abort(self.MOTOR_SPINUP_WAIT):
                self.get_logger().warn('Launch aborted before first ball.')
                return

            # 4. Launch first ball
            if not self.launch_one_ball(1):
                return

            # 5. Wait 2 sec, launch second
            self.get_logger().info(f'Waiting {self.DELAY_AFTER_FIRST:.1f}s before second ball...')
            if not self.sleep_with_abort(self.DELAY_AFTER_FIRST):
                self.get_logger().warn('Launch aborted before second ball.')
                return

            if not self.launch_one_ball(2):
                return

            # 6. Wait 8 sec, launch third
            self.get_logger().info(f'Waiting {self.DELAY_AFTER_SECOND:.1f}s before third ball...')
            if not self.sleep_with_abort(self.DELAY_AFTER_SECOND):
                self.get_logger().warn('Launch aborted before third ball.')
                return

            if not self.launch_one_ball(3):
                return

            # 7. Power off motor
            self.motor_stop()
            self.get_logger().info('Motor powered off. Launch sequence complete.')

            # 8. Publish done
            done = Bool()
            done.data = True
            self.launch_done_pub.publish(done)

            self.has_launched = True
            self.get_logger().info('Published /launch_done = True')

        except Exception as e:
            self.get_logger().error(f'Error during launch sequence: {e}')

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
            self.get_logger().info('GPIO cleaned up.')
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaunchingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
