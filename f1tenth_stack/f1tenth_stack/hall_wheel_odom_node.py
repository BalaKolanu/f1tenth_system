#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2026
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
# THE SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
# THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import math
import shutil
import subprocess
import threading
import time
from collections import deque

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import UInt64
from tf2_ros import TransformBroadcaster


class HallWheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('hall_wheel_odom_node')

        self.declare_parameter('chip', 'gpiochip0')
        self.declare_parameter('line_offset', 43)
        self.declare_parameter('edge', 'falling')
        self.declare_parameter('bias', 'as-is')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('speed_timeout_sec', 0.25)
        self.declare_parameter('velocity_window_sec', 0.5)
        self.declare_parameter('speed_lowpass_alpha', 0.25)
        self.declare_parameter('pulses_per_revolution', 1.0)
        self.declare_parameter('wheel_diameter_m', 0.110)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('speed_topic', '/sensors/wheel_speed_mps')
        self.declare_parameter('count_topic', '/sensors/wheel_count')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('forward_sign', 1.0)

        self._chip = str(self.get_parameter('chip').value)
        self._line_offset = int(self.get_parameter('line_offset').value)
        self._edge = str(self.get_parameter('edge').value).strip().lower()
        self._bias = str(self.get_parameter('bias').value).strip().lower()
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._speed_timeout_sec = float(self.get_parameter('speed_timeout_sec').value)
        self._velocity_window_sec = float(self.get_parameter('velocity_window_sec').value)
        self._speed_lowpass_alpha = float(self.get_parameter('speed_lowpass_alpha').value)
        self._pulses_per_revolution = float(self.get_parameter('pulses_per_revolution').value)
        self._wheel_diameter_m = float(self.get_parameter('wheel_diameter_m').value)
        self._gear_ratio = float(self.get_parameter('gear_ratio').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._speed_topic = str(self.get_parameter('speed_topic').value)
        self._count_topic = str(self.get_parameter('count_topic').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._forward_sign = float(self.get_parameter('forward_sign').value)

        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 50.0
        if self._speed_timeout_sec <= 0.0:
            self._speed_timeout_sec = 0.25
        if self._velocity_window_sec <= 0.0:
            self._velocity_window_sec = 0.5
        if self._speed_lowpass_alpha < 0.0:
            self._speed_lowpass_alpha = 0.0
        if self._speed_lowpass_alpha > 1.0:
            self._speed_lowpass_alpha = 1.0
        if self._pulses_per_revolution <= 0.0:
            raise ValueError('pulses_per_revolution must be > 0')
        if self._wheel_diameter_m <= 0.0:
            raise ValueError('wheel_diameter_m must be > 0')
        if self._gear_ratio <= 0.0:
            raise ValueError('gear_ratio must be > 0')
        if self._edge not in ('rising', 'falling', 'both'):
            raise ValueError("edge must be one of: 'rising', 'falling', 'both'")
        if self._bias not in ('as-is', 'pull-up', 'pull-down', 'disable'):
            raise ValueError(
                "bias must be one of: 'as-is', 'pull-up', 'pull-down', 'disable'"
            )
        if self._forward_sign == 0.0:
            self._forward_sign = 1.0
        self._forward_sign = 1.0 if self._forward_sign > 0.0 else -1.0

        self._meters_per_pulse = (
            math.pi * self._wheel_diameter_m / (self._pulses_per_revolution * self._gear_ratio)
        )

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 20)
        self._speed_pub = self.create_publisher(Float64, self._speed_topic, 20)
        self._count_pub = self.create_publisher(UInt64, self._count_topic, 20)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self._event_times = deque()
        self._count = 0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_event_time = None
        self._filtered_speed_mps = 0.0
        self._filtered_speed_initialized = False
        self._timestamp_parse_warned = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._process = None
        self._reader_thread = None

        gpiomon_path = shutil.which('gpiomon')
        if gpiomon_path is None:
            raise RuntimeError('gpiomon is required but was not found in PATH')

        cmd = [gpiomon_path, '-b', '-F', '%s %n', f'--bias={self._bias}']
        if self._edge == 'rising':
            cmd.append('--rising-edge')
        elif self._edge == 'falling':
            cmd.append('--falling-edge')
        cmd.extend([self._chip, str(self._line_offset)])

        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

        self._reader_thread = threading.Thread(target=self._read_events, daemon=True)
        self._reader_thread.start()

        self.create_timer(1.0 / self._publish_rate_hz, self._publish_odom)

        self.get_logger().info(
            'Hall wheel odom active: chip=%s line=%d edge=%s bias=%s odom_topic=%s '
            'meters_per_pulse=%.6f'
            % (
                self._chip,
                self._line_offset,
                self._edge,
                self._bias,
                self._odom_topic,
                self._meters_per_pulse,
            )
        )

    @staticmethod
    def _parse_event_time_sec(text: str):
        parts = text.split()
        if len(parts) < 2:
            return None
        try:
            sec = int(parts[0])
            nsec = int(parts[1])
        except ValueError:
            return None
        return float(sec) + (float(nsec) * 1e-9)

    def _read_events(self) -> None:
        assert self._process is not None
        assert self._process.stdout is not None

        for line in self._process.stdout:
            if self._stop_event.is_set():
                break

            text = line.strip()
            if not text:
                continue

            event_time_sec = self._parse_event_time_sec(text)
            if event_time_sec is None:
                event_time_sec = time.monotonic()
                if not self._timestamp_parse_warned:
                    self._timestamp_parse_warned = True
                    self.get_logger().warning(
                        'Failed to parse gpiomon event timestamps, falling back to local receive time'
                    )

            with self._lock:
                self._count += 1
                self._last_event_time = event_time_sec
                self._event_times.append(event_time_sec)
                cutoff = event_time_sec - self._velocity_window_sec
                while self._event_times and self._event_times[0] < cutoff:
                    self._event_times.popleft()

        if not self._stop_event.is_set():
            stderr_output = ''
            if self._process.stderr is not None:
                stderr_output = self._process.stderr.read().strip()
            if stderr_output:
                self.get_logger().error('gpiomon stopped: %s' % stderr_output)
            else:
                self.get_logger().warning('gpiomon stopped without error output')

    def _compute_speed_locked(self, now_sec: float) -> float:
        if self._last_event_time is None:
            return 0.0
        if now_sec - self._last_event_time > self._speed_timeout_sec:
            return 0.0
        if len(self._event_times) < 2:
            return self._forward_sign * self._meters_per_pulse / self._speed_timeout_sec

        dt = self._event_times[-1] - self._event_times[0]
        if dt <= 1e-6:
            return 0.0

        pulse_count = float(len(self._event_times) - 1)
        pulses_per_sec = pulse_count / dt
        return self._forward_sign * pulses_per_sec * self._meters_per_pulse

    def _publish_odom(self) -> None:
        now_monotonic = time.monotonic()
        with self._lock:
            raw_speed_mps = self._compute_speed_locked(now_monotonic)
            count = self._count

        if self._filtered_speed_initialized:
            alpha = self._speed_lowpass_alpha
            speed_mps = alpha * raw_speed_mps + (1.0 - alpha) * self._filtered_speed_mps
        else:
            speed_mps = raw_speed_mps
            self._filtered_speed_initialized = True
        self._filtered_speed_mps = speed_mps

        # Position from accumulated counts is much more stable than integrating
        # a coarse pulse-timing speed estimate at the publish rate.
        self._x = self._forward_sign * float(count) * self._meters_per_pulse

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odom.pose.covariance[0] = 0.5
        odom.pose.covariance[7] = 0.5
        odom.pose.covariance[14] = 99999.0
        odom.pose.covariance[21] = 99999.0
        odom.pose.covariance[28] = 99999.0
        odom.pose.covariance[35] = 99999.0
        odom.twist.twist.linear.x = speed_mps
        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance[0] = 0.2
        odom.twist.covariance[7] = 99999.0
        odom.twist.covariance[14] = 99999.0
        odom.twist.covariance[21] = 99999.0
        odom.twist.covariance[28] = 99999.0
        odom.twist.covariance[35] = 99999.0
        self._odom_pub.publish(odom)

        speed_msg = Float64()
        speed_msg.data = speed_mps
        self._speed_pub.publish(speed_msg)

        count_msg = UInt64()
        count_msg.data = count
        self._count_pub.publish(count_msg)

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._odom_frame
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self) -> bool:
        self._stop_event.set()

        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)

        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HallWheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
