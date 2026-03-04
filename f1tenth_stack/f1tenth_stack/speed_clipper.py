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
from std_msgs.msg import Float64


class SpeedClipper(Node):
    def __init__(self):
        super().__init__('speed_clipper')

        self.declare_parameter('input_topic', 'commands/motor/unclipped_speed')
        self.declare_parameter('output_topic', 'commands/motor/speed')
        self.declare_parameter('min_value', -1800.0)
        self.declare_parameter('max_value', 1800.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.min_value = float(self.get_parameter('min_value').value)
        self.max_value = float(self.get_parameter('max_value').value)

        if self.min_value > self.max_value:
            self.get_logger().warn(
                'min_value > max_value; swapping values '
                f'({self.min_value}, {self.max_value})'
            )
            self.min_value, self.max_value = self.max_value, self.min_value

        self.publisher = self.create_publisher(Float64, output_topic, 10)
        self.subscription = self.create_subscription(
            Float64,
            input_topic,
            self.speed_callback,
            10,
        )

        self.get_logger().info(
            'Clipping motor speed from "%s" to "%s" in [%0.2f, %0.2f]'
            % (input_topic, output_topic, self.min_value, self.max_value)
        )

    def speed_callback(self, msg: Float64):
        out = Float64()
        out.data = min(max(msg.data, self.min_value), self.max_value)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedClipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
