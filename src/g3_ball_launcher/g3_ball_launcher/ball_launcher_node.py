#!/usr/bin/env python3
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# Import the UART servo SDK (bundled inside this package under uart_sdk/)
from .uart_sdk.uart_servo import UartServoManager
from .uart_sdk.data_table import MOTOR_MODE_DC, MOTOR_MODE_SERVO, DC_DIR_CW

# --- Hardware config ---
SERVO_PORT = '/dev/ttyUSB0'  # USB serial port the servo is connected to
SERVO_BAUD = 115200          # Must match servo's configured baud rate
SERVO_ID = 1                 # ID of the launcher servo on the bus
FIRE_DURATION = 2.2          # How long (seconds) to spin the motor per ball
HOME_POSITION = 2048         # Midpoint (0-4095) to return to after each shot


class BallLauncherNode(Node):
    def __init__(self):
        super().__init__('ball_launcher')

        # Tracks current launcher state. Mission controller reads this via /launcher_status.
        # Valid values: 'idle' | 'firing' | 'complete'
        self.status = 'idle'

        # Lock prevents a second fire command from interfering while _fire_thread is running
        self._lock = threading.Lock()

        # --- UART servo setup ---
        uart = serial.Serial(
            port=SERVO_PORT, baudrate=SERVO_BAUD,
            parity=serial.PARITY_NONE, stopbits=1,
            bytesize=8, timeout=0  # timeout=0 = non-blocking reads
        )
        # UartServoManager wraps the serial port and handles packet framing
        self.uservo = UartServoManager(uart, servo_id_list=[SERVO_ID])
        # Switch servo from position (servo) mode to DC motor mode so we can spin it freely
        self.uservo.set_motor_mode(SERVO_ID, MOTOR_MODE_DC)

        # --- ROS interfaces ---
        # Publishes 'idle'/'firing'/'complete' so mission controller knows launcher state
        self.status_pub = self.create_publisher(String, '/launcher_status', 10)
        # Receives True from mission controller to trigger one ball
        self.create_subscription(Bool, '/fire_launcher', self.fire_cb, 10)
        # Broadcast status at 10 Hz so mission controller always has a fresh reading
        self.create_timer(0.1, self.publish_status)

        self.get_logger().info('Ball launcher node ready.')

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)

    def fire_cb(self, msg: Bool):
        # Ignore False values (mission controller only sends True, but be safe)
        if not msg.data:
            return

        # Only accept a fire command if we're idle — prevents double-firing
        with self._lock:
            if self.status != 'idle':
                return
            self.status = 'firing'  # claim the launcher before releasing the lock

        self.get_logger().info('Firing ball...')

        # Run the motor in a background thread so we don't block ROS callbacks
        # (time.sleep inside a callback would freeze the whole node)
        threading.Thread(target=self._fire_thread, daemon=True).start()

    def _fire_thread(self):
        # Spin motor clockwise at full speed (PWM = 100%)
        self.uservo.dc_rotate(SERVO_ID, DC_DIR_CW, 100)

        # Keep spinning for FIRE_DURATION seconds to launch the ball
        time.sleep(FIRE_DURATION)

        # Stop the motor
        self.uservo.dc_stop(SERVO_ID)

        # Signal to mission controller that this ball is done
        with self._lock:
            self.status = 'complete'
        self.publish_status()  # push 'complete' immediately, don't wait for the 10 Hz timer

        # Brief pause so mission controller has time to see 'complete' before we go idle
        time.sleep(0.1)

        # # Switch back to servo mode so we can command a specific position
        # self.uservo.set_motor_mode(SERVO_ID, MOTOR_MODE_SERVO)
        # time.sleep(0.1)  # small delay for mode switch to settle

        # # Read where the servo ended up and move it back to home position
        # current_pos = self.uservo.read_data_by_name(SERVO_ID, "CURRENT_POSITION")
        # self.get_logger().info(f'Post-fire position: {current_pos}, correcting to {HOME_POSITION}')
        # self.uservo.write_data_by_name(SERVO_ID, "TARGET_POSITION", HOME_POSITION)

        # # Switch back to DC mode ready for the next shot
        # time.sleep(0.5)  # give servo time to reach home before switching modes
        # self.uservo.set_motor_mode(SERVO_ID, MOTOR_MODE_DC)

        with self._lock:
            self.status = 'idle'
        self.get_logger().info('Correction done, launcher idle.')


def main(args=None):
    rclpy.init(args=args)
    node = BallLauncherNode()
    rclpy.spin(node)   # blocks here, running callbacks until Ctrl+C
    rclpy.shutdown()


if __name__ == '__main__':
    main()
