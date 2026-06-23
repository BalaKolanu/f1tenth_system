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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class SpeedClipper(Node):
    def __init__(self):
        super().__init__('speed_clipper')

        self.declare_parameter('input_topic', 'commands/motor/unclipped_speed')
        self.declare_parameter('output_topic', 'commands/motor/speed')
        self.declare_parameter('min_value', -1800.0)
        self.declare_parameter('max_value', 1800.0)
        self.declare_parameter('joy_topic', '')
        self.declare_parameter('speed_clip_axis_index', 7)
        self.declare_parameter('speed_clip_step_erpm', 500.0)
        self.declare_parameter('runtime_min_abs_cap_erpm', 0.0)
        self.declare_parameter('runtime_max_abs_cap_erpm', 0.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.min_value = float(self.get_parameter('min_value').value)
        self.max_value = float(self.get_parameter('max_value').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.speed_clip_axis_index = int(self.get_parameter('speed_clip_axis_index').value)
        self.speed_clip_step_erpm = max(
            1.0, float(self.get_parameter('speed_clip_step_erpm').value)
        )
        runtime_min_abs_cap_erpm = float(
            self.get_parameter('runtime_min_abs_cap_erpm').value
        )
        runtime_max_abs_cap_erpm = float(
            self.get_parameter('runtime_max_abs_cap_erpm').value
        )

        if self.min_value > self.max_value:
            self.get_logger().warn(
                'min_value > max_value; swapping values '
                f'({self.min_value}, {self.max_value})'
            )
            self.min_value, self.max_value = self.max_value, self.min_value

        initial_abs_cap = max(0.0, min(abs(self.min_value), abs(self.max_value)))
        self.runtime_min_abs_cap_erpm = max(0.0, runtime_min_abs_cap_erpm)
        if self.runtime_min_abs_cap_erpm <= 0.0:
            self.runtime_min_abs_cap_erpm = initial_abs_cap
        self.runtime_abs_cap_erpm = max(self.runtime_min_abs_cap_erpm, initial_abs_cap)
        self.runtime_max_abs_cap_erpm = (
            max(self.runtime_min_abs_cap_erpm, runtime_max_abs_cap_erpm)
            if runtime_max_abs_cap_erpm > 0.0
            else None
        )
        self._last_speed_clip_axis_dir = 0
        self._apply_runtime_cap()

        self.publisher = self.create_publisher(Float64, output_topic, 10)
        self.subscription = self.create_subscription(
            Float64,
            input_topic,
            self.speed_callback,
            10,
        )
        self.joy_subscription = None
        if self.joy_topic:
            self.joy_subscription = self.create_subscription(
                Joy,
                self.joy_topic,
                self.joy_callback,
                10,
            )

        self.get_logger().info(
            'Clipping motor speed from "%s" to "%s" in [%0.2f, %0.2f]; runtime joy cap=%s'
            % (
                input_topic,
                output_topic,
                self.min_value,
                self.max_value,
                self.joy_topic if self.joy_topic else 'disabled',
            )
        )

    def speed_callback(self, msg: Float64):
        out = Float64()
        out.data = min(max(msg.data, self.min_value), self.max_value)
        self.publisher.publish(out)

    def _apply_runtime_cap(self):
        cap = max(self.runtime_min_abs_cap_erpm, self.runtime_abs_cap_erpm)
        if self.runtime_max_abs_cap_erpm is not None:
            cap = min(cap, self.runtime_max_abs_cap_erpm)
        self.runtime_abs_cap_erpm = cap
        self.min_value = -cap
        self.max_value = cap

    def joy_callback(self, msg: Joy):
        axis_dir = 0
        if 0 <= self.speed_clip_axis_index < len(msg.axes):
            axis_val = float(msg.axes[self.speed_clip_axis_index])
            if axis_val > 0.5:
                axis_dir = 1
            elif axis_val < -0.5:
                axis_dir = -1

        if axis_dir != 0 and axis_dir != self._last_speed_clip_axis_dir:
            self.runtime_abs_cap_erpm += axis_dir * self.speed_clip_step_erpm
            self._apply_runtime_cap()
            self.get_logger().info(
                'Updated runtime speed cap from axis %d: abs_cap_erpm=%.0f'
                % (self.speed_clip_axis_index, self.runtime_abs_cap_erpm)
            )
        self._last_speed_clip_axis_dir = axis_dir


def main(args=None):
    rclpy.init(args=args)
    node = SpeedClipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
