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
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfSpeedMonitor(Node):
    def __init__(self):
        super().__init__('tf_speed_monitor')

        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('speed_mps_topic', '/analysis/tf_speed_mps')
        self.declare_parameter('speed_kmph_topic', '/analysis/tf_speed_kmph')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('lookup_timeout_sec', 0.05)
        self.declare_parameter('max_dt_sec', 0.25)
        self.declare_parameter('max_position_jump_m', 0.75)
        self.declare_parameter('lowpass_alpha', 0.35)

        self.source_frame = str(self.get_parameter('source_frame').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        speed_mps_topic = str(self.get_parameter('speed_mps_topic').value)
        speed_kmph_topic = str(self.get_parameter('speed_kmph_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.lookup_timeout_sec = float(self.get_parameter('lookup_timeout_sec').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.max_position_jump_m = float(self.get_parameter('max_position_jump_m').value)
        self.lowpass_alpha = float(self.get_parameter('lowpass_alpha').value)

        if publish_hz <= 0.0:
            publish_hz = 30.0
        if self.lookup_timeout_sec <= 0.0:
            self.lookup_timeout_sec = 0.05
        if self.max_dt_sec <= 0.0:
            self.max_dt_sec = 0.25
        if self.max_position_jump_m <= 0.0:
            self.max_position_jump_m = 0.75
        if self.lowpass_alpha < 0.0:
            self.lowpass_alpha = 0.0
        if self.lowpass_alpha > 1.0:
            self.lowpass_alpha = 1.0

        self.pub_mps = self.create_publisher(Float32, speed_mps_topic, 20)
        self.pub_kmph = self.create_publisher(Float32, speed_kmph_topic, 20)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_x = None
        self.prev_y = None
        self.prev_stamp_sec = None
        self.filtered_speed_mps = 0.0
        self.have_filtered_speed = False
        self.last_jump_warn_time = 0.0

        self.timer = self.create_timer(1.0 / publish_hz, self.handle_timer)

        self.get_logger().info(
            'TF speed monitor active: %s->%s mps_topic=%s kmph_topic=%s hz=%.1f alpha=%.2f'
            % (
                self.source_frame,
                self.target_frame,
                speed_mps_topic,
                speed_kmph_topic,
                publish_hz,
                self.lowpass_alpha,
            )
        )

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _publish_speed(self, speed_mps):
        speed_mps = max(0.0, float(speed_mps))

        msg_mps = Float32()
        msg_mps.data = float(speed_mps)
        self.pub_mps.publish(msg_mps)

        msg_kmph = Float32()
        msg_kmph.data = float(speed_mps * 3.6)
        self.pub_kmph.publish(msg_kmph)

    def handle_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
        except TransformException:
            return

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        stamp_sec = self._stamp_to_seconds(transform.header.stamp)

        if self.prev_stamp_sec is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_stamp_sec = stamp_sec
            self._publish_speed(0.0)
            return

        dt = stamp_sec - self.prev_stamp_sec
        if dt <= 1e-6:
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        step_m = math.hypot(dx, dy)

        self.prev_x = x
        self.prev_y = y
        self.prev_stamp_sec = stamp_sec

        if dt > self.max_dt_sec:
            return

        if step_m > self.max_position_jump_m:
            now = time.monotonic()
            if now - self.last_jump_warn_time > 1.0:
                self.last_jump_warn_time = now
                self.get_logger().warn(
                    'Ignoring TF jump: step=%.3fm dt=%.3fs (>%.3fm)'
                    % (step_m, dt, self.max_position_jump_m)
                )
            return

        speed_raw = step_m / dt
        if not self.have_filtered_speed:
            self.filtered_speed_mps = speed_raw
            self.have_filtered_speed = True
        else:
            a = self.lowpass_alpha
            self.filtered_speed_mps = a * speed_raw + (1.0 - a) * self.filtered_speed_mps

        self._publish_speed(self.filtered_speed_mps)


def main(args=None):
    rclpy.init(args=args)
    node = TfSpeedMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
