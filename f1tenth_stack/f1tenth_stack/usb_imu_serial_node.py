#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2026 Bala Kolanu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import os
import re
import select
import termios
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu


class UsbImuSerialNode(Node):
    _FLOAT_PATTERN = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
    _SERIAL_PATTERN = re.compile(
        rf'RPY\(deg\):\s*({_FLOAT_PATTERN})\s*,\s*({_FLOAT_PATTERN})\s*,\s*({_FLOAT_PATTERN})'
        rf'\s*\|\s*AccelDir\(world\):\s*({_FLOAT_PATTERN})\s*,\s*({_FLOAT_PATTERN})\s*,\s*({_FLOAT_PATTERN})'
    )
    _BAUDRATE_MAP = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        921600: termios.B921600,
    }

    def __init__(self):
        super().__init__('usb_imu_serial_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('frame_id', 'imu_frame')
        self.declare_parameter('accel_scale', 9.80665)
        self.declare_parameter('orientation_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.5)
        self.declare_parameter('angular_velocity_covariance', -1.0)
        self.declare_parameter('poll_period_sec', 0.01)
        self.declare_parameter('reconnect_period_sec', 1.0)
        self.declare_parameter('max_buffer_chars', 8192)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.accel_scale = float(self.get_parameter('accel_scale').value)
        orientation_cov = float(self.get_parameter('orientation_covariance').value)
        linear_accel_cov = float(self.get_parameter('linear_acceleration_covariance').value)
        angular_vel_cov = float(self.get_parameter('angular_velocity_covariance').value)
        poll_period = float(self.get_parameter('poll_period_sec').value)
        self.reconnect_period_sec = max(0.1, float(self.get_parameter('reconnect_period_sec').value))
        self.max_buffer_chars = max(512, int(self.get_parameter('max_buffer_chars').value))

        if poll_period <= 0.0:
            poll_period = 0.01

        self.orientation_covariance = self._build_covariance(orientation_cov)
        self.linear_acceleration_covariance = self._build_covariance(linear_accel_cov)
        self.angular_velocity_covariance = self._build_covariance(angular_vel_cov)

        self.imu_pub = self.create_publisher(Imu, imu_topic, QoSPresetProfiles.SENSOR_DATA.value)

        self._fd = None
        self._buffer = ''
        self._last_open_attempt = 0.0
        self._last_warn = {}

        self.create_timer(poll_period, self._poll_serial)

        self.get_logger().info(
            'USB IMU serial node configured. port=%s baudrate=%d topic=%s'
            % (self.port, self.baudrate, imu_topic)
        )

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()

    def _build_covariance(self, value: float) -> List[float]:
        if value < 0.0:
            return [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return [value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value]

    def _warn_throttled(self, key: str, message: str, period_sec: float = 5.0):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= period_sec:
            self._last_warn[key] = now
            self.get_logger().warning(message)

    def _poll_serial(self):
        if self._fd is None:
            self._try_open_serial()
            return

        try:
            ready, _, _ = select.select([self._fd], [], [], 0.0)
        except OSError as exc:
            self._warn_throttled('select_error', 'Serial select failed: %s' % str(exc))
            self._close_serial()
            return

        if not ready:
            return

        while True:
            try:
                raw = os.read(self._fd, 4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self._warn_throttled('read_error', 'Serial read failed: %s' % str(exc))
                self._close_serial()
                return

            if not raw:
                break

            self._buffer += raw.decode('utf-8', errors='ignore')

        self._consume_buffer()

    def _try_open_serial(self):
        now = time.monotonic()
        if now - self._last_open_attempt < self.reconnect_period_sec:
            return
        self._last_open_attempt = now

        try:
            fd = os.open(self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            self._configure_serial(fd, self.baudrate)
            self._fd = fd
            self._buffer = ''
            self.get_logger().info('Connected to serial IMU on %s' % self.port)
        except (OSError, ValueError) as exc:
            self._warn_throttled(
                'open_error',
                'Unable to open IMU serial port %s: %s' % (self.port, str(exc)),
                period_sec=self.reconnect_period_sec,
            )

    def _close_serial(self):
        if self._fd is None:
            return
        try:
            os.close(self._fd)
        except OSError:
            pass
        self._fd = None

    def _configure_serial(self, fd: int, baudrate: int):
        speed = self._BAUDRATE_MAP.get(baudrate)
        if speed is None:
            supported = sorted(self._BAUDRATE_MAP.keys())
            raise ValueError(
                'Unsupported baudrate %d. Supported: %s' % (baudrate, ', '.join(str(v) for v in supported))
            )

        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
        attrs[2] |= termios.CS8

        if hasattr(termios, 'cfsetispeed') and hasattr(termios, 'cfsetospeed'):
            termios.cfsetispeed(attrs, speed)
            termios.cfsetospeed(attrs, speed)
        else:
            attrs[4] = speed
            attrs[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)

    def _consume_buffer(self):
        while True:
            match = self._SERIAL_PATTERN.search(self._buffer)
            if match is None:
                self._trim_buffer()
                return

            if match.start() > 0:
                self._buffer = self._buffer[match.start():]
                match = self._SERIAL_PATTERN.search(self._buffer)
                if match is None:
                    self._trim_buffer()
                    return

            try:
                roll_deg, pitch_deg, yaw_deg, ax_dir, ay_dir, az_dir = (float(v) for v in match.groups())
            except ValueError:
                self._buffer = self._buffer[match.end():]
                continue

            self._buffer = self._buffer[match.end():]
            self._publish_imu(roll_deg, pitch_deg, yaw_deg, ax_dir, ay_dir, az_dir)

    def _trim_buffer(self):
        if len(self._buffer) <= self.max_buffer_chars:
            return

        marker_idx = self._buffer.rfind('RPY(deg):')
        if marker_idx >= 0:
            self._buffer = self._buffer[marker_idx:]

        if len(self._buffer) > self.max_buffer_chars:
            self._buffer = self._buffer[-self.max_buffer_chars:]

    def _publish_imu(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        ax_dir: float,
        ay_dir: float,
        az_dir: float,
    ):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = self._rpy_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.linear_acceleration.x = ax_dir * self.accel_scale
        msg.linear_acceleration.y = ay_dir * self.accel_scale
        msg.linear_acceleration.z = az_dir * self.accel_scale

        msg.orientation_covariance = self.orientation_covariance
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        msg.angular_velocity_covariance = self.angular_velocity_covariance

        self.imu_pub.publish(msg)

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = UsbImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
