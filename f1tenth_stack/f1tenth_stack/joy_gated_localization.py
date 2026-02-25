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

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class JoyGatedLocalization(Node):
    def __init__(self):
        super().__init__('joy_gated_localization')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('enable_button_index', 0)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_localization')
        self.declare_parameter('input_pose_topic', '/scanmatching_localization/pose_raw')
        self.declare_parameter('output_pose_topic', '/scanmatching_localization/pose')
        self.declare_parameter('enabled_topic', '/scanmatching_localization/enabled')

        self.enable_button_index = int(self.get_parameter('enable_button_index').value)
        joy_timeout_sec = float(self.get_parameter('joy_timeout_sec').value)
        if joy_timeout_sec < 0.0:
            joy_timeout_sec = 0.0
        self.joy_timeout = Duration(seconds=joy_timeout_sec)

        joy_topic = str(self.get_parameter('joy_topic').value)
        input_scan_topic = str(self.get_parameter('input_scan_topic').value)
        output_scan_topic = str(self.get_parameter('output_scan_topic').value)
        input_pose_topic = str(self.get_parameter('input_pose_topic').value)
        output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        enabled_topic = str(self.get_parameter('enabled_topic').value)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.scan_publisher = self.create_publisher(LaserScan, output_scan_topic, sensor_qos)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, output_pose_topic, 10)
        self.enabled_publisher = self.create_publisher(Bool, enabled_topic, 10)

        self.create_subscription(Joy, joy_topic, self.handle_joy, sensor_qos)
        self.create_subscription(LaserScan, input_scan_topic, self.handle_scan, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            input_pose_topic,
            self.handle_pose,
            10,
        )

        self.enabled = False
        self.last_joy_msg_time = None
        self.create_timer(0.1, self.check_joy_timeout)
        self.publish_enabled_state()

        self.get_logger().info(
            'Localization scan gating ready. Hold joystick button index %d '
            'to enable scan matching.' % self.enable_button_index
        )

    def handle_joy(self, msg: Joy):
        self.last_joy_msg_time = self.get_clock().now()

        pressed = False
        if 0 <= self.enable_button_index < len(msg.buttons):
            pressed = msg.buttons[self.enable_button_index] == 1

        self.set_enabled(pressed)

    def handle_scan(self, msg: LaserScan):
        if self.is_enabled():
            self.scan_publisher.publish(msg)

    def handle_pose(self, msg: PoseWithCovarianceStamped):
        self.pose_publisher.publish(msg)

    def check_joy_timeout(self):
        if not self.enabled:
            return
        if self.joy_timeout.nanoseconds == 0:
            return
        if self.last_joy_msg_time is None:
            self.set_enabled(False)
            return
        if (self.get_clock().now() - self.last_joy_msg_time) > self.joy_timeout:
            self.set_enabled(False)

    def is_enabled(self):
        if not self.enabled:
            return False
        if self.joy_timeout.nanoseconds == 0:
            return True
        if self.last_joy_msg_time is None:
            return False
        return (self.get_clock().now() - self.last_joy_msg_time) <= self.joy_timeout

    def set_enabled(self, enabled: bool):
        if enabled == self.enabled:
            return
        self.enabled = enabled
        state_msg = 'ENABLED' if enabled else 'DISABLED'
        self.get_logger().info('Scan matching gate: %s' % state_msg)
        self.publish_enabled_state()

    def publish_enabled_state(self):
        msg = Bool()
        msg.data = bool(self.enabled)
        self.enabled_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyGatedLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
