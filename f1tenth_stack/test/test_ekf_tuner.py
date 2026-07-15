"""Synthetic-motion test for PF-referenced EKF tuning estimates."""

# Copyright (c) 2026 Purdue University
# SPDX-License-Identifier: MIT

import math

from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import Imu

from f1tenth_stack.ekf_tuner_node import EkfTunerNode


def _set_stamp(message, seconds):
    nanoseconds = int(round(seconds * 1e9))
    message.header.stamp.sec = nanoseconds // 1_000_000_000
    message.header.stamp.nanosec = nanoseconds % 1_000_000_000


def _set_yaw(orientation, yaw):
    orientation.z = math.sin(0.5 * yaw)
    orientation.w = math.cos(0.5 * yaw)


def test_tuner_recovers_bias_and_wheel_scale_from_pf_motion():
    rclpy.init()
    node = EkfTunerNode()
    try:
        speed = 1.0
        yaw_rate = 0.2
        gyro_bias = 0.04
        wheel_scale = 1.25
        x = 0.0
        y = 0.0
        yaw = 0.0
        dt = 0.1
        for index in range(120):
            stamp = 1.0 + index * dt
            midpoint_yaw = yaw + 0.5 * yaw_rate * dt
            x += speed * math.cos(midpoint_yaw) * dt
            y += speed * math.sin(midpoint_yaw) * dt
            yaw += yaw_rate * dt

            imu = Imu()
            _set_stamp(imu, stamp)
            imu.angular_velocity.z = yaw_rate + gyro_bias
            imu.linear_acceleration.x = 0.0
            _set_yaw(imu.orientation, yaw)
            node._handle_imu(imu)

            wheel = Odometry()
            _set_stamp(wheel, stamp)
            wheel.twist.twist.linear.x = speed / wheel_scale
            node._handle_wheel(wheel)

            ekf = Odometry()
            _set_stamp(ekf, stamp)
            ekf.pose.pose.position.x = x
            ekf.pose.pose.position.y = y
            _set_yaw(ekf.pose.pose.orientation, yaw)
            node._handle_ekf(ekf)

            particle_filter = Odometry()
            _set_stamp(particle_filter, stamp)
            particle_filter.pose.pose.position.x = x
            particle_filter.pose.pose.position.y = y
            _set_yaw(particle_filter.pose.pose.orientation, yaw)
            node._handle_pf(particle_filter)

        assert node.gyro_bias.count > 100
        assert abs(node.gyro_bias.mean - gyro_bias) < 0.01
        assert node.wheel_scale.count > 100
        assert abs(node.wheel_scale.mean - wheel_scale) < 0.02
        assert node.ekf_position_error.rms < 1e-6
    finally:
        node.destroy_node()
        rclpy.shutdown()
