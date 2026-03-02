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

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def _norm_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ImuOdomFusionNode(Node):
    def __init__(self):
        super().__init__('imu_odom_fusion_node')

        self.declare_parameter('imu_topic', '/sensors/imu')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('fused_odom_topic', '/odometry/imu_fused')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('linear_speed_scale', 1.0)
        self.declare_parameter('use_first_imu_as_zero', True)
        self.declare_parameter('yaw_offset_rad', 0.0)
        self.declare_parameter('yaw_alpha', 1.0)
        self.declare_parameter('max_dt_sec', 0.2)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.fused_odom_topic = str(self.get_parameter('fused_odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.linear_speed_scale = float(self.get_parameter('linear_speed_scale').value)
        self.use_first_imu_as_zero = bool(self.get_parameter('use_first_imu_as_zero').value)
        self.yaw_offset_rad = float(self.get_parameter('yaw_offset_rad').value)
        self.yaw_alpha = float(self.get_parameter('yaw_alpha').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)

        if self.yaw_alpha < 0.0:
            self.yaw_alpha = 0.0
        if self.yaw_alpha > 1.0:
            self.yaw_alpha = 1.0
        if self.max_dt_sec <= 0.0:
            self.max_dt_sec = 0.2

        self.imu_yaw_ref = None
        self.imu_yaw = 0.0
        self.have_imu = False

        self.initialized = False
        self.last_odom_time = None
        self.x = 0.0
        self.y = 0.0
        self.prev_yaw = 0.0
        self.latest_orientation_covariance = [0.0] * 9

        self.odom_pub = self.create_publisher(Odometry, self.fused_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, self.imu_topic, self.handle_imu, sensor_qos)
        self.create_subscription(Odometry, self.wheel_odom_topic, self.handle_wheel_odom, 50)

        self.get_logger().info(
            'IMU odom fusion active: imu=%s wheel_odom=%s fused_odom=%s publish_tf=%s'
            % (self.imu_topic, self.wheel_odom_topic, self.fused_odom_topic, str(self.publish_tf))
        )

    def handle_imu(self, msg):
        raw_yaw = _yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        if self.imu_yaw_ref is None:
            self.imu_yaw_ref = raw_yaw if self.use_first_imu_as_zero else 0.0

        measured = _norm_angle(raw_yaw - self.imu_yaw_ref + self.yaw_offset_rad)

        if self.have_imu and self.yaw_alpha < 1.0:
            delta = _norm_angle(measured - self.imu_yaw)
            self.imu_yaw = _norm_angle(self.imu_yaw + self.yaw_alpha * delta)
        else:
            self.imu_yaw = measured

        self.have_imu = True
        self.latest_orientation_covariance = list(msg.orientation_covariance)

    def handle_wheel_odom(self, msg):
        current_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if not self.initialized:
            self.x = float(msg.pose.pose.position.x)
            self.y = float(msg.pose.pose.position.y)
            initial_yaw = (
                self.imu_yaw
                if self.have_imu
                else _yaw_from_quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                )
            )
            self.prev_yaw = initial_yaw
            self.last_odom_time = current_sec
            self.initialized = True
            self.publish_fused(msg, initial_yaw, 0.0)
            return

        dt = current_sec - self.last_odom_time
        self.last_odom_time = current_sec
        if dt <= 0.0:
            return
        if dt > self.max_dt_sec:
            dt = self.max_dt_sec

        yaw = (
            self.imu_yaw
            if self.have_imu
            else _yaw_from_quaternion(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
        )

        v = float(msg.twist.twist.linear.x) * self.linear_speed_scale
        self.x += v * dt * math.cos(yaw)
        self.y += v * dt * math.sin(yaw)

        yaw_rate = _norm_angle(yaw - self.prev_yaw) / dt
        self.prev_yaw = yaw

        self.publish_fused(msg, yaw, yaw_rate)

    def publish_fused(self, wheel_odom_msg, yaw, yaw_rate):
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)

        msg = Odometry()
        msg.header.stamp = wheel_odom_msg.header.stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = list(wheel_odom_msg.pose.covariance)
        if len(msg.pose.covariance) >= 36:
            yaw_cov = self.latest_orientation_covariance[8] if self.have_imu else 0.2
            if yaw_cov < 0.0:
                yaw_cov = 0.2
            msg.pose.covariance[35] = yaw_cov

        msg.twist.twist = wheel_odom_msg.twist.twist
        msg.twist.twist.angular.z = yaw_rate
        msg.twist.covariance = list(wheel_odom_msg.twist.covariance)

        self.odom_pub.publish(msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
