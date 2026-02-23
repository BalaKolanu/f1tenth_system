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

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ScanMatchingTfPublisher(Node):

    def __init__(self):
        super().__init__('scanmatching_tf_publisher')

        self.declare_parameter('init_source_frame_name', 'map')
        self.declare_parameter('target_frame_name', 'base_link')
        self.declare_parameter('init_tf_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('updater_topic', '/scanmatching_odom/pose')
        self.declare_parameter('min_tf_broadcast_frequency', 40.0)

        self.tf = TransformStamped()
        self.tf.header.frame_id = self.get_parameter('init_source_frame_name').value
        self.tf.child_frame_id = self.get_parameter('target_frame_name').value

        tf_init = self.get_parameter('init_tf_pose').value
        self.tf.transform.translation.x = float(tf_init[0])
        self.tf.transform.translation.y = float(tf_init[1])
        self.tf.transform.translation.z = float(tf_init[2])
        qx, qy, qz, qw = quaternion_from_euler(tf_init[3], tf_init[4], tf_init[5])
        self.tf.transform.rotation.x = qx
        self.tf.transform.rotation.y = qy
        self.tf.transform.rotation.z = qz
        self.tf.transform.rotation.w = qw

        self.tf_broadcaster = TransformBroadcaster(self)

        updater_topic = self.get_parameter('updater_topic').value
        self.create_subscription(
            PoseWithCovarianceStamped,
            updater_topic,
            self.handle_tf_update,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        tf_frequency = float(self.get_parameter('min_tf_broadcast_frequency').value)
        if tf_frequency <= 0.0:
            tf_frequency = 10.0
        self.send_tf_timer = self.create_timer(1.0 / tf_frequency, self.send_tf_callback)

    def send_tf_callback(self):
        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf)

    def handle_tf_update(self, msg):
        if msg.header.frame_id:
            self.tf.header.frame_id = msg.header.frame_id

        pos = msg.pose.pose.position
        self.tf.transform.translation.x = pos.x
        self.tf.transform.translation.y = pos.y
        self.tf.transform.translation.z = pos.z
        self.tf.transform.rotation = msg.pose.pose.orientation

        # Push the latest scan-matching estimate immediately.
        self.send_tf_timer.reset()
        self.send_tf_callback()


def main():
    rclpy.init()
    node = ScanMatchingTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
