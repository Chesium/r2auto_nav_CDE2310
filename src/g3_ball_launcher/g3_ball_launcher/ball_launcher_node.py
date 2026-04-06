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
        # Valid values: 'idle' | 'firing' | 'complete' | 'error'
        self.status = 'error'

        # Lock prevents a second fire command from interfering while _fire_thread is running
        self._lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._stop_requested = threading.Event()

        self.uart = None
        self.uservo = None

        self.declare_parameter('servo_port', SERVO_PORT)
        self.declare_parameter('servo_baud', SERVO_BAUD)
        self.declare_parameter('servo_id', SERVO_ID)
        self.declare_parameter('fire_duration', FIRE_DURATION)

        self.servo_port = str(self.get_parameter('servo_port').value)
        self.servo_baud = int(self.get_parameter('servo_baud').value)
        self.servo_id = int(self.get_parameter('servo_id').value)
        self.fire_duration = float(self.get_parameter('fire_duration').value)

        # --- ROS interfaces ---
        # Publishes 'idle'/'firing'/'complete' so mission controller knows launcher state
        self.status_pub = self.create_publisher(String, '/launcher_status', 10)
        # Receives True from mission controller to trigger one ball
        self.create_subscription(Bool, '/fire_launcher', self.fire_cb, 10)
        # Broadcast status at 10 Hz so mission controller always has a fresh reading
        self.create_timer(0.1, self.publish_status)
        # Retry serial connection in the background so a replugged device can recover
        self.create_timer(1.0, self._ensure_servo_connection)

        self._connect_servo()

        self.get_logger().info('Ball launcher node ready.')

    def _close_uart(self):
        if self.uart is None:
            return

        try:
            if self.uart.is_open:
                self.uart.close()
        except serial.SerialException:
            pass

        self.uart = None
        self.uservo = None

    def _handle_serial_failure(self, exc, context):
        self._close_uart()
        with self._lock:
            self.status = 'error'
        self.get_logger().error(
            f'{context}: {exc}. Check {self.servo_port}, USB connection, and that no other process is using the port.'
        )

    def _stop_motor(self, reason='Stopping launcher motor'):
        if self.uservo is None:
            return False

        self.get_logger().info(reason)
        with self._serial_lock:
            self.uservo.dc_stop(self.servo_id)
            time.sleep(0.02)
            self.uservo.dc_stop(self.servo_id)
        return True

    def _manual_stop_thread(self):
        if self.uservo is None and not self._connect_servo():
            return

        try:
            if self._stop_motor('Manual stop requested for launcher motor'):
                with self._lock:
                    self.status = 'idle'
                self.publish_status()
        except serial.SerialException as exc:
            self._handle_serial_failure(exc, 'Launcher serial communication failed during manual stop')
            return

        self._stop_requested.clear()
        self.get_logger().info('Launcher manual stop completed.')

    def _connect_servo(self):
        if self.uservo is not None:
            return True

        self._close_uart()
        try:
            uart = serial.Serial(
                port=self.servo_port,
                baudrate=self.servo_baud,
                parity=serial.PARITY_NONE,
                stopbits=1,
                bytesize=8,
                timeout=0,
                exclusive=True,
            )
            uservo = UartServoManager(uart, servo_id_list=[self.servo_id])
            # Must set servo mode first before switching to DC mode.
            uservo.set_motor_mode(self.servo_id, MOTOR_MODE_DC)
            time.sleep(0.1)
            uservo.torque_enable(self.servo_id, True)
            time.sleep(0.1)
            # Force PWM to zero in case a previous run left the motor spinning.
            uservo.dc_stop(self.servo_id)
        except serial.SerialException as exc:
            self._handle_serial_failure(exc, 'Unable to initialize launcher serial link')
            return False

        self.uart = uart
        self.uservo = uservo
        with self._lock:
            self.status = 'idle'
        self.get_logger().info(
            f'Connected to launcher servo on {self.servo_port} at {self.servo_baud} baud.'
        )
        return True

    def _ensure_servo_connection(self):
        if self.uservo is None:
            self._connect_servo()

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)

    def fire_cb(self, msg: Bool):
        if not msg.data:
            with self._lock:
                is_firing = self.status == 'firing'
            self._stop_requested.set()
            if is_firing:
                self.get_logger().warning('Received manual stop request for launcher.')
            else:
                self.get_logger().info('Received stop request while launcher was not firing.')
            threading.Thread(target=self._manual_stop_thread, daemon=True).start()
            return

        # Only accept a fire command if we're idle — prevents double-firing
        with self._lock:
            if self.status != 'idle':
                if self.status == 'error':
                    self.get_logger().warning('Ignoring fire command because launcher serial link is unavailable.')
                return
            self._stop_requested.clear()
            self.status = 'firing'  # claim the launcher before releasing the lock

        self.get_logger().info('Firing ball...')

        # Run the motor in a background thread so we don't block ROS callbacks
        # (time.sleep inside a callback would freeze the whole node)
        threading.Thread(target=self._fire_thread, daemon=True).start()

    def _fire_thread(self):
        if self.uservo is None and not self._connect_servo():
            return

        try:
            # Spin motor clockwise at full speed (PWM = 100%)
            with self._serial_lock:
                self.uservo.dc_rotate(self.servo_id, DC_DIR_CW, 100)

            # Keep spinning for FIRE_DURATION seconds to launch the ball
            t_start = time.time()
            while time.time() - t_start < self.fire_duration:
                if self._stop_requested.wait(timeout=0.05):
                    break

            # Stop the motor
            self._stop_motor()
        except serial.SerialException as exc:
            self._handle_serial_failure(exc, 'Launcher serial communication failed during firing')
            return

        if self._stop_requested.is_set():
            with self._lock:
                self.status = 'idle'
            self._stop_requested.clear()
            self.get_logger().info('Launcher stop request completed.')
            return

        # Signal to mission controller that this ball is done
        with self._lock:
            self.status = 'complete'
        self.publish_status()  # push 'complete' immediately, don't wait for the 10 Hz timer

        # Brief pause so mission controller has time to see 'complete' before we go idle
        time.sleep(0.1)

        # # Switch back to servo mode so we can command a specific position
        # self.uservo.set_motor_mode(self.servo_id, MOTOR_MODE_SERVO)
        # time.sleep(0.1)  # small delay for mode switch to settle

        # # Read where the servo ended up and move it back to home position
        # current_pos = self.uservo.read_data_by_name(self.servo_id, "CURRENT_POSITION")
        # self.get_logger().info(f'Post-fire position: {current_pos}, correcting to {HOME_POSITION}')
        # self.uservo.write_data_by_name(self.servo_id, "TARGET_POSITION", HOME_POSITION)

        # # Switch back to DC mode ready for the next shot
        # time.sleep(0.5)  # give servo time to reach home before switching modes
        # self.uservo.set_motor_mode(self.servo_id, MOTOR_MODE_DC)

        with self._lock:
            self.status = 'idle'
        self.get_logger().info('Correction done, launcher idle.')

    def destroy_node(self):
        try:
            self._stop_motor('Stopping launcher motor before shutdown')
        except serial.SerialException:
            pass
        self._close_uart()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallLauncherNode()
    rclpy.spin(node)   # blocks here, running callbacks until Ctrl+C
    rclpy.shutdown()


if __name__ == '__main__':
    main()
