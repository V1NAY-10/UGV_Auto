#!/usr/bin/env python3
"""Bridge /cmd_vel to the ESP32 motor controller over the Pi's GPIO UART.

The ESP32 drives two L298N boards (4 non-encoded DC motors, skid steer).
Each cycle this node sends one line:  "<left_pwm>,<right_pwm>\n"
with PWM values in [-255, 255] (sign = direction). The ESP32 stops the
motors by itself if no line arrives for 500 ms.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        self.port = self.declare_parameter('port', '/dev/ttyAMA0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.20).value
        self.turn_gain = self.declare_parameter('turn_gain', 1.6).value
        self.max_wheel_speed = self.declare_parameter('max_wheel_speed', 1.2).value
        self.min_pwm = self.declare_parameter('min_pwm', 90).value
        self.max_pwm = self.declare_parameter('max_pwm', 255).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        rate = self.declare_parameter('rate', 20.0).value

        self.ser = None
        self.open_serial()

        self.left_pwm = 0
        self.right_pwm = 0
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / rate, self.send)

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'Connected to ESP32 on {self.port} @ {self.baud}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(f'Cannot open {self.port}: {e}', throttle_duration_sec=5.0)

    def speed_to_pwm(self, speed):
        """Map wheel speed (m/s) to signed PWM, jumping over the motor deadband."""
        if abs(speed) < 1e-3:
            return 0
        frac = min(abs(speed) / self.max_wheel_speed, 1.0)
        pwm = self.min_pwm + frac * (self.max_pwm - self.min_pwm)
        return int(math.copysign(round(pwm), speed))

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z * self.turn_gain
        left = v - w * self.wheel_separation / 2.0
        right = v + w * self.wheel_separation / 2.0
        self.left_pwm = self.speed_to_pwm(left)
        self.right_pwm = self.speed_to_pwm(right)
        self.last_cmd_time = self.get_clock().now()

    def send(self):
        if self.ser is None:
            self.open_serial()
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.left_pwm = 0
            self.right_pwm = 0

        try:
            self.ser.write(f'{self.left_pwm},{self.right_pwm}\n'.encode())
            # Drain any debug text the ESP32 prints so the buffer never fills
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial error: {e}')
            self.ser.close()
            self.ser = None

    def stop(self):
        if self.ser is not None:
            try:
                self.ser.write(b'0,0\n')
            except (serial.SerialException, OSError):
                pass


def main():
    rclpy.init()
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
