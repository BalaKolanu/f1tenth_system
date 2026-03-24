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
from typing import List, Optional, Tuple

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


def _clamp(value, low, high):
    return max(low, min(high, value))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _rpy_from_quaternion(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    yaw = _yaw_from_quaternion(x, y, z, w)
    return (roll, pitch, yaw)


def _quaternion_from_rpy(roll, pitch, yaw):
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw

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


def _normalize_quaternion(
    quat: Tuple[float, float, float, float]
) -> Optional[Tuple[float, float, float, float]]:
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        return None
    return (x / norm, y / norm, z / norm, w / norm)


def _rotate_vector(
    quat: Tuple[float, float, float, float], vector: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    x, y, z, w = quat
    vx, vy, vz = vector

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
    )


class ImuOdomFusionNode(Node):
    def __init__(self):
        super().__init__('imu_odom_fusion_node')

        self.declare_parameter('imu_topic', '/sensors/imu/raw')
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
        self.declare_parameter('use_accel_fusion', True)
        self.declare_parameter('wheel_speed_alpha', 0.85)
        self.declare_parameter('accel_lowpass_alpha', 0.25)
        self.declare_parameter('zero_velocity_speed_threshold', 0.05)
        self.declare_parameter('zero_velocity_accel_threshold', 0.25)
        self.declare_parameter('use_imu_angular_velocity', True)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.fused_odom_topic = str(self.get_parameter('fused_odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = _as_bool(self.get_parameter('publish_tf').value)
        self.linear_speed_scale = float(self.get_parameter('linear_speed_scale').value)
        self.use_first_imu_as_zero = _as_bool(self.get_parameter('use_first_imu_as_zero').value)
        self.yaw_offset_rad = float(self.get_parameter('yaw_offset_rad').value)
        self.yaw_alpha = float(self.get_parameter('yaw_alpha').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.use_accel_fusion = _as_bool(self.get_parameter('use_accel_fusion').value)
        self.wheel_speed_alpha = float(self.get_parameter('wheel_speed_alpha').value)
        self.accel_lowpass_alpha = float(self.get_parameter('accel_lowpass_alpha').value)
        self.zero_velocity_speed_threshold = float(self.get_parameter('zero_velocity_speed_threshold').value)
        self.zero_velocity_accel_threshold = float(self.get_parameter('zero_velocity_accel_threshold').value)
        self.use_imu_angular_velocity = _as_bool(self.get_parameter('use_imu_angular_velocity').value)

        if self.yaw_alpha < 0.0:
            self.yaw_alpha = 0.0
        if self.yaw_alpha > 1.0:
            self.yaw_alpha = 1.0
        if self.max_dt_sec <= 0.0:
            self.max_dt_sec = 0.2
        self.wheel_speed_alpha = _clamp(self.wheel_speed_alpha, 0.0, 1.0)
        self.accel_lowpass_alpha = _clamp(self.accel_lowpass_alpha, 0.0, 1.0)
        if self.zero_velocity_speed_threshold < 0.0:
            self.zero_velocity_speed_threshold = 0.0
        if self.zero_velocity_accel_threshold < 0.0:
            self.zero_velocity_accel_threshold = 0.0

        self.imu_yaw_ref = None
        self.imu_yaw = 0.0
        self.have_imu_orientation = False
        self.latest_orientation_quaternion = (0.0, 0.0, 0.0, 1.0)
        self.latest_orientation_covariance = [0.0] * 9
        self.latest_linear_acceleration = (0.0, 0.0, 0.0)
        self.have_linear_acceleration = False
        self.latest_angular_velocity_z = 0.0
        self.have_angular_velocity = False
        self.latest_angular_velocity_covariance = [0.0] * 9

        self.initialized = False
        self.last_odom_time = None
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.prev_yaw = 0.0
        self._last_warn = {}

        self.odom_pub = self.create_publisher(Odometry, self.fused_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, self.imu_topic, self.handle_imu, sensor_qos)
        self.create_subscription(Odometry, self.wheel_odom_topic, self.handle_wheel_odom, 50)

        self.get_logger().info(
            'IMU odom fusion active: imu=%s wheel_odom=%s fused_odom=%s '
            'publish_tf=%s use_accel_fusion=%s wheel_speed_alpha=%.2f'
            % (
                self.imu_topic,
                self.wheel_odom_topic,
                self.fused_odom_topic,
                str(self.publish_tf),
                str(self.use_accel_fusion),
                self.wheel_speed_alpha,
            )
        )

    def _warn_throttled(self, key, message, period_sec=5.0):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= period_sec:
            self._last_warn[key] = now
            self.get_logger().warning(message)

    @staticmethod
    def _covariance_available(covariance: List[float]) -> bool:
        return len(covariance) > 0 and covariance[0] >= 0.0

    def handle_imu(self, msg):
        raw_quat = _normalize_quaternion(
            (
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            )
        )
        if raw_quat is None:
            self._warn_throttled('imu_orientation_invalid', 'Received invalid IMU orientation quaternion')
            return

        raw_roll, raw_pitch, raw_yaw = _rpy_from_quaternion(
            raw_quat[0],
            raw_quat[1],
            raw_quat[2],
            raw_quat[3],
        )

        if self.imu_yaw_ref is None:
            self.imu_yaw_ref = raw_yaw if self.use_first_imu_as_zero else 0.0

        measured_yaw = _norm_angle(raw_yaw - self.imu_yaw_ref + self.yaw_offset_rad)

        if self.have_imu_orientation and self.yaw_alpha < 1.0:
            delta = _norm_angle(measured_yaw - self.imu_yaw)
            self.imu_yaw = _norm_angle(self.imu_yaw + self.yaw_alpha * delta)
        else:
            self.imu_yaw = measured_yaw

        self.latest_orientation_quaternion = _quaternion_from_rpy(raw_roll, raw_pitch, self.imu_yaw)
        self.latest_orientation_covariance = list(msg.orientation_covariance)
        self.have_imu_orientation = True

        accel = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )
        if all(math.isfinite(v) for v in accel) and self._covariance_available(list(msg.linear_acceleration_covariance)):
            if self.have_linear_acceleration:
                alpha = self.accel_lowpass_alpha
                self.latest_linear_acceleration = (
                    (1.0 - alpha) * self.latest_linear_acceleration[0] + alpha * accel[0],
                    (1.0 - alpha) * self.latest_linear_acceleration[1] + alpha * accel[1],
                    (1.0 - alpha) * self.latest_linear_acceleration[2] + alpha * accel[2],
                )
            else:
                self.latest_linear_acceleration = accel
            self.have_linear_acceleration = True

        angular_velocity_z = float(msg.angular_velocity.z)
        if (
            math.isfinite(angular_velocity_z)
            and self._covariance_available(list(msg.angular_velocity_covariance))
        ):
            self.latest_angular_velocity_z = angular_velocity_z
            self.latest_angular_velocity_covariance = list(msg.angular_velocity_covariance)
            self.have_angular_velocity = True

    def handle_wheel_odom(self, msg):
        current_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        wheel_quat = _normalize_quaternion(
            (
                float(msg.pose.pose.orientation.x),
                float(msg.pose.pose.orientation.y),
                float(msg.pose.pose.orientation.z),
                float(msg.pose.pose.orientation.w),
            )
        )
        if wheel_quat is None:
            wheel_quat = (0.0, 0.0, 0.0, 1.0)

        fused_quat = self.latest_orientation_quaternion if self.have_imu_orientation else wheel_quat
        _, _, fused_yaw = _rpy_from_quaternion(
            fused_quat[0], fused_quat[1], fused_quat[2], fused_quat[3]
        )
        wheel_speed = float(msg.twist.twist.linear.x) * self.linear_speed_scale

        if not self.initialized:
            self.x = float(msg.pose.pose.position.x)
            self.y = float(msg.pose.pose.position.y)
            self.vx = wheel_speed * math.cos(fused_yaw)
            self.vy = wheel_speed * math.sin(fused_yaw)
            self.prev_yaw = fused_yaw
            self.last_odom_time = current_sec
            self.initialized = True
            self.publish_fused(msg, fused_quat, wheel_speed, 0.0)
            return

        dt = current_sec - self.last_odom_time
        self.last_odom_time = current_sec
        if dt <= 0.0:
            return
        if dt > self.max_dt_sec:
            dt = self.max_dt_sec

        heading_x = math.cos(fused_yaw)
        heading_y = math.sin(fused_yaw)
        wheel_vx = wheel_speed * heading_x
        wheel_vy = wheel_speed * heading_y

        if self.use_accel_fusion and self.have_imu_orientation and self.have_linear_acceleration:
            world_accel = _rotate_vector(self.latest_orientation_quaternion, self.latest_linear_acceleration)
            pred_vx = self.vx + world_accel[0] * dt
            pred_vy = self.vy + world_accel[1] * dt
            alpha = self.wheel_speed_alpha
            self.vx = alpha * wheel_vx + (1.0 - alpha) * pred_vx
            self.vy = alpha * wheel_vy + (1.0 - alpha) * pred_vy

            accel_mag = math.sqrt(
                self.latest_linear_acceleration[0] * self.latest_linear_acceleration[0]
                + self.latest_linear_acceleration[1] * self.latest_linear_acceleration[1]
                + self.latest_linear_acceleration[2] * self.latest_linear_acceleration[2]
            )
            if (
                abs(wheel_speed) <= self.zero_velocity_speed_threshold
                and accel_mag <= self.zero_velocity_accel_threshold
            ):
                self.vx = 0.0
                self.vy = 0.0
        else:
            self.vx = wheel_vx
            self.vy = wheel_vy

        self.x += self.vx * dt
        self.y += self.vy * dt

        fused_speed = self.vx * heading_x + self.vy * heading_y
        if self.use_imu_angular_velocity and self.have_angular_velocity:
            yaw_rate = self.latest_angular_velocity_z
        else:
            yaw_rate = _norm_angle(fused_yaw - self.prev_yaw) / dt
        self.prev_yaw = fused_yaw

        self.publish_fused(msg, fused_quat, fused_speed, yaw_rate)

    def publish_fused(self, wheel_odom_msg, orientation_quat, fused_speed, yaw_rate):
        qx, qy, qz, qw = orientation_quat

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
        if len(msg.pose.covariance) >= 36 and self.have_imu_orientation:
            roll_cov = self.latest_orientation_covariance[0]
            pitch_cov = self.latest_orientation_covariance[4]
            yaw_cov = self.latest_orientation_covariance[8]
            if roll_cov >= 0.0:
                msg.pose.covariance[21] = roll_cov
            if pitch_cov >= 0.0:
                msg.pose.covariance[28] = pitch_cov
            if yaw_cov >= 0.0:
                msg.pose.covariance[35] = yaw_cov

        msg.twist.twist = wheel_odom_msg.twist.twist
        msg.twist.twist.linear.x = fused_speed
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = yaw_rate
        msg.twist.covariance = list(wheel_odom_msg.twist.covariance)
        if len(msg.twist.covariance) >= 36 and self.have_angular_velocity:
            angular_cov = self.latest_angular_velocity_covariance[8]
            if angular_cov >= 0.0:
                msg.twist.covariance[35] = angular_cov

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
