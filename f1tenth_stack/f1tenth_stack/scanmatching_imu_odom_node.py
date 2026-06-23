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
from functools import partial
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _norm_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _normalize_quaternion(
    quat: Tuple[float, float, float, float]
) -> Optional[Tuple[float, float, float, float]]:
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        return None
    return (x / norm, y / norm, z / norm, w / norm)


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


def _rpy_from_quaternion(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def _quaternion_conjugate(
    quat: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    x, y, z, w = quat
    return (-x, -y, -z, w)


def _quaternion_multiply(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


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


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _clamp(value, low, high):
    return max(low, min(high, value))


def _weighted_angle_average(
    weighted_angles: List[Tuple[float, float]]
) -> Optional[float]:
    if not weighted_angles:
        return None

    sin_sum = 0.0
    cos_sum = 0.0
    for angle, weight in weighted_angles:
        if weight <= 0.0 or not math.isfinite(angle):
            continue
        sin_sum += weight * math.sin(angle)
        cos_sum += weight * math.cos(angle)

    if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
        return weighted_angles[0][0]
    return math.atan2(sin_sum, cos_sum)


class ScanmatchingImuOdomNode(Node):
    def __init__(self):
        super().__init__('scanmatching_imu_odom_node')

        self.declare_parameter('pose_topic', '/scanmatching_tf/pose')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('output_topic', '/odometry/scanmatched_imu')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_frame')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('yaw_offset_rad', 0.0)
        self.declare_parameter('yaw_alpha', 1.0)
        self.declare_parameter('use_imu_angular_velocity', True)
        self.declare_parameter('secondary_imu_topic', '')
        self.declare_parameter('secondary_imu_frame_id', '')
        self.declare_parameter('secondary_imu_weight', 0.35)
        self.declare_parameter('secondary_imu_angular_velocity_weight', 0.5)
        self.declare_parameter('secondary_imu_timeout_sec', 0.25)
        self.declare_parameter('dual_imu_max_yaw_delta_rad', 0.35)
        self.declare_parameter('dual_imu_outlier_weight', 0.1)
        self.declare_parameter('imu_enabled_topic', '')
        self.declare_parameter('startup_imu_enabled', True)
        self.declare_parameter('prefer_secondary_imu_on_yaw_disagreement', False)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.publish_tf = _as_bool(self.get_parameter('publish_tf').value)
        self.yaw_offset_rad = float(self.get_parameter('yaw_offset_rad').value)
        self.yaw_alpha = float(self.get_parameter('yaw_alpha').value)
        self.use_imu_angular_velocity = _as_bool(
            self.get_parameter('use_imu_angular_velocity').value
        )
        self.secondary_imu_topic = str(self.get_parameter('secondary_imu_topic').value)
        self.secondary_imu_frame_id = str(self.get_parameter('secondary_imu_frame_id').value)
        self.secondary_imu_weight = float(self.get_parameter('secondary_imu_weight').value)
        self.secondary_imu_angular_velocity_weight = float(
            self.get_parameter('secondary_imu_angular_velocity_weight').value
        )
        self.secondary_imu_timeout_sec = float(self.get_parameter('secondary_imu_timeout_sec').value)
        self.dual_imu_max_yaw_delta_rad = float(
            self.get_parameter('dual_imu_max_yaw_delta_rad').value
        )
        self.dual_imu_outlier_weight = float(
            self.get_parameter('dual_imu_outlier_weight').value
        )
        self.imu_enabled_topic = str(self.get_parameter('imu_enabled_topic').value)
        self.imu_enabled = _as_bool(self.get_parameter('startup_imu_enabled').value)
        self.prefer_secondary_imu_on_yaw_disagreement = _as_bool(
            self.get_parameter('prefer_secondary_imu_on_yaw_disagreement').value
        )

        if self.yaw_alpha < 0.0:
            self.yaw_alpha = 0.0
        if self.yaw_alpha > 1.0:
            self.yaw_alpha = 1.0
        self.secondary_imu_weight = _clamp(self.secondary_imu_weight, 0.0, 1.0)
        self.secondary_imu_angular_velocity_weight = _clamp(
            self.secondary_imu_angular_velocity_weight, 0.0, 1.0
        )
        if self.secondary_imu_timeout_sec <= 0.0:
            self.secondary_imu_timeout_sec = 0.25
        if self.dual_imu_max_yaw_delta_rad <= 0.0:
            self.dual_imu_max_yaw_delta_rad = 0.35
        self.dual_imu_outlier_weight = _clamp(self.dual_imu_outlier_weight, 0.0, 1.0)

        self.have_imu_orientation = False
        self.have_angular_velocity = False
        self.latest_imu_roll = 0.0
        self.latest_imu_pitch = 0.0
        self.latest_imu_yaw = 0.0
        self.latest_map_yaw = 0.0
        self.latest_orientation_covariance = [0.0] * 9
        self.latest_angular_velocity_z = 0.0
        self.latest_angular_velocity_covariance = [0.0] * 9
        self.map_yaw_bias = None
        self.last_pose_xy = None
        self.last_pose_time = None
        self.last_output_yaw = None
        self._imu_to_base_quat_cache = {}
        self._last_warn = {}
        self.imu_states: Dict[str, Dict[str, object]] = {
            'primary': self._make_imu_state(self.imu_frame_id, 1.0, 1.0),
        }
        if self.secondary_imu_topic:
            secondary_frame = self.secondary_imu_frame_id or self.base_frame
            self.imu_states['secondary'] = self._make_imu_state(
                secondary_frame,
                self.secondary_imu_weight,
                self.secondary_imu_angular_velocity_weight,
            )

        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(
            Imu,
            self.imu_topic,
            partial(self.handle_imu, sensor_key='primary'),
            sensor_qos,
        )
        if self.secondary_imu_topic:
            self.create_subscription(
                Imu,
                self.secondary_imu_topic,
                partial(self.handle_imu, sensor_key='secondary'),
                sensor_qos,
            )
        if self.imu_enabled_topic:
            self.create_subscription(
                Bool,
                self.imu_enabled_topic,
                self.handle_imu_enabled,
                10,
            )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.handle_pose,
            20,
        )

        self.get_logger().info(
            'Scan-matching + IMU odom active: pose=%s imu=%s secondary_imu=%s output=%s publish_tf=%s'
            % (
                self.pose_topic,
                self.imu_topic,
                self.secondary_imu_topic or '(disabled)',
                self.output_topic,
                str(self.publish_tf),
            )
        )

    def handle_imu_enabled(self, msg: Bool):
        enabled = bool(msg.data)
        if enabled == self.imu_enabled:
            return

        self.imu_enabled = enabled
        if enabled:
            # Re-align IMU yaw to the current scan-matching heading on re-enable.
            self.map_yaw_bias = None
        state = 'ENABLED' if enabled else 'DISABLED'
        self.get_logger().info('Mapping IMU fusion: %s' % state)

    @staticmethod
    def _make_imu_state(frame_id: str, orientation_weight: float, angular_velocity_weight: float):
        return {
            'frame_id': frame_id,
            'orientation_weight': orientation_weight,
            'angular_velocity_weight': angular_velocity_weight,
            'yaw_bias': None,
            'measured_yaw': 0.0,
            'raw_roll': 0.0,
            'raw_pitch': 0.0,
            'latest_orientation_covariance': [0.0] * 9,
            'have_orientation': False,
            'latest_angular_velocity_z': 0.0,
            'latest_angular_velocity_covariance': [0.0] * 9,
            'have_angular_velocity': False,
            'last_msg_time_sec': None,
        }

    def _warn_throttled(self, key, message, period_sec=5.0):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= period_sec:
            self._last_warn[key] = now
            self.get_logger().warning(message)

    @staticmethod
    def _covariance_available(covariance: List[float]) -> bool:
        return len(covariance) > 0 and covariance[0] >= 0.0

    def _lookup_imu_to_base_quaternion(
        self, source_frame: str
    ) -> Optional[Tuple[float, float, float, float]]:
        if not source_frame or source_frame == self.base_frame:
            return (0.0, 0.0, 0.0, 1.0)

        cached_quat = self._imu_to_base_quat_cache.get(source_frame)
        if cached_quat is not None:
            return cached_quat

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                'imu_tf_missing',
                'Unable to transform IMU frame %s into %s yet: %s'
                % (source_frame, self.base_frame, str(exc)),
            )
            return None

        quat = _normalize_quaternion(
            (
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            )
        )
        if quat is None:
            self._warn_throttled(
                'imu_tf_invalid',
                'Received invalid base_link<-imu_frame quaternion for frame %s'
                % source_frame,
            )
            return None

        self._imu_to_base_quat_cache[source_frame] = quat
        return quat

    def _imu_state_is_fresh(self, state: Dict[str, object], current_time_sec: float) -> bool:
        last_msg_time_sec = state['last_msg_time_sec']
        if last_msg_time_sec is None:
            return False
        if current_time_sec <= 0.0:
            return True
        return abs(current_time_sec - float(last_msg_time_sec)) <= self.secondary_imu_timeout_sec

    def _orientation_candidates(self, current_time_sec: float):
        candidates = []
        for sensor_key, state in self.imu_states.items():
            if not state['have_orientation'] or not self._imu_state_is_fresh(state, current_time_sec):
                continue
            candidates.append(
                {
                    'sensor_key': sensor_key,
                    'yaw': float(state['measured_yaw']),
                    'weight': float(state['orientation_weight']),
                    'roll': float(state['raw_roll']),
                    'pitch': float(state['raw_pitch']),
                    'covariance': list(state['latest_orientation_covariance']),
                }
            )
        return candidates

    def _angular_velocity_candidates(self, current_time_sec: float):
        candidates = []
        for state in self.imu_states.values():
            if not state['have_angular_velocity'] or not self._imu_state_is_fresh(state, current_time_sec):
                continue
            candidates.append(
                (
                    float(state['latest_angular_velocity_z']),
                    float(state['angular_velocity_weight']),
                    list(state['latest_angular_velocity_covariance']),
                )
            )
        return candidates

    def _apply_dual_imu_outlier_guard(self, candidates, warn_key: str, warn_message: str):
        if len(candidates) < 2:
            return candidates

        yaw_delta = abs(_norm_angle(candidates[0]['yaw'] - candidates[1]['yaw']))
        if yaw_delta <= self.dual_imu_max_yaw_delta_rad:
            return candidates

        if self.prefer_secondary_imu_on_yaw_disagreement:
            preferred_index = next(
                (
                    idx
                    for idx, candidate in enumerate(candidates)
                    if candidate.get('sensor_key') == 'secondary'
                ),
                None,
            )
        else:
            preferred_index = None

        if preferred_index is None and self.have_imu_orientation:
            preferred_index = min(
                range(len(candidates)),
                key=lambda idx: abs(_norm_angle(candidates[idx]['yaw'] - self.latest_imu_yaw)),
            )
        elif preferred_index is None:
            preferred_index = max(
                range(len(candidates)),
                key=lambda idx: candidates[idx]['weight'],
            )

        adjusted = []
        for idx, candidate in enumerate(candidates):
            adjusted_candidate = dict(candidate)
            if idx != preferred_index:
                adjusted_candidate['weight'] *= self.dual_imu_outlier_weight
            adjusted.append(adjusted_candidate)

        self._warn_throttled(warn_key, warn_message)
        return adjusted

    def _update_fused_imu_state(self, current_time_sec: float):
        orientation_candidates = self._apply_dual_imu_outlier_guard(
            self._orientation_candidates(current_time_sec),
            'dual_imu_yaw_disagreement',
            (
                'Primary and secondary IMUs disagree on yaw by more than %.1f deg; '
                'downweighting the outlier sensor'
            )
            % math.degrees(self.dual_imu_max_yaw_delta_rad),
        )
        if orientation_candidates:
            weighted_yaw = _weighted_angle_average(
                [(candidate['yaw'], candidate['weight']) for candidate in orientation_candidates]
            )
            if weighted_yaw is not None:
                dominant = max(orientation_candidates, key=lambda candidate: candidate['weight'])
                self.latest_imu_yaw = weighted_yaw
                self.latest_imu_roll = dominant['roll']
                self.latest_imu_pitch = dominant['pitch']
                self.latest_orientation_covariance = dominant['covariance']
                self.have_imu_orientation = True

        angular_velocity_candidates = self._angular_velocity_candidates(current_time_sec)
        if angular_velocity_candidates:
            total_weight = sum(weight for _, weight, _ in angular_velocity_candidates)
            if total_weight > 1e-6:
                self.latest_angular_velocity_z = sum(
                    value * weight for value, weight, _ in angular_velocity_candidates
                ) / total_weight
                self.latest_angular_velocity_covariance = max(
                    angular_velocity_candidates,
                    key=lambda candidate: candidate[1],
                )[2]
                self.have_angular_velocity = True
        elif len(self.imu_states) > 1:
            self.have_angular_velocity = False

    def handle_imu(self, msg: Imu, sensor_key='primary'):
        state = self.imu_states[sensor_key]
        fallback_frame = str(state['frame_id'])
        source_frame = msg.header.frame_id if msg.header.frame_id else fallback_frame
        base_from_imu = self._lookup_imu_to_base_quaternion(source_frame)
        if base_from_imu is None:
            return

        raw_quat = _normalize_quaternion(
            (
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            )
        )
        if raw_quat is None:
            self._warn_throttled(
                'imu_orientation_invalid',
                'Received invalid IMU orientation quaternion',
            )
            return

        base_quat = _normalize_quaternion(
            _quaternion_multiply(raw_quat, _quaternion_conjugate(base_from_imu))
        )
        if base_quat is None:
            self._warn_throttled(
                'imu_orientation_base_invalid',
                'Unable to compute base frame orientation from IMU quaternion',
            )
            return

        raw_roll, raw_pitch, raw_yaw = _rpy_from_quaternion(
            base_quat[0],
            base_quat[1],
            base_quat[2],
            base_quat[3],
        )

        if state['yaw_bias'] is None:
            if self.have_imu_orientation:
                state['yaw_bias'] = _norm_angle(self.latest_imu_yaw - raw_yaw)
            else:
                state['yaw_bias'] = self.yaw_offset_rad

        state['measured_yaw'] = _norm_angle(raw_yaw + float(state['yaw_bias']))
        state['raw_roll'] = raw_roll
        state['raw_pitch'] = raw_pitch
        state['latest_orientation_covariance'] = list(msg.orientation_covariance)
        state['have_orientation'] = True
        state['last_msg_time_sec'] = _stamp_to_sec(msg.header.stamp)

        angular_velocity = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        if all(math.isfinite(v) for v in angular_velocity) and self._covariance_available(
            list(msg.angular_velocity_covariance)
        ):
            base_angular_velocity = _rotate_vector(base_from_imu, angular_velocity)
            state['latest_angular_velocity_z'] = base_angular_velocity[2]
            state['latest_angular_velocity_covariance'] = list(
                msg.angular_velocity_covariance
            )
            state['have_angular_velocity'] = True

        self._update_fused_imu_state(float(state['last_msg_time_sec']))

    def handle_pose(self, msg: PoseWithCovarianceStamped):
        pose_quat = _normalize_quaternion(
            (
                float(msg.pose.pose.orientation.x),
                float(msg.pose.pose.orientation.y),
                float(msg.pose.pose.orientation.z),
                float(msg.pose.pose.orientation.w),
            )
        )
        if pose_quat is None:
            pose_quat = (0.0, 0.0, 0.0, 1.0)
        _, _, pose_yaw = _rpy_from_quaternion(
            pose_quat[0],
            pose_quat[1],
            pose_quat[2],
            pose_quat[3],
        )

        imu_fusion_active = self.imu_enabled and self.have_imu_orientation

        if imu_fusion_active and self.map_yaw_bias is None:
            self.map_yaw_bias = _norm_angle(pose_yaw - self.latest_imu_yaw)

        if imu_fusion_active and self.map_yaw_bias is not None:
            measured_map_yaw = _norm_angle(self.latest_imu_yaw + self.map_yaw_bias)
            if self.last_output_yaw is not None and self.yaw_alpha < 1.0:
                delta = _norm_angle(measured_map_yaw - self.latest_map_yaw)
                self.latest_map_yaw = _norm_angle(
                    self.latest_map_yaw + self.yaw_alpha * delta
                )
            else:
                self.latest_map_yaw = measured_map_yaw
            qx, qy, qz, qw = _quaternion_from_rpy(
                self.latest_imu_roll,
                self.latest_imu_pitch,
                self.latest_map_yaw,
            )
            output_quat = (qx, qy, qz, qw)
        else:
            output_quat = pose_quat
            _, _, self.latest_map_yaw = _rpy_from_quaternion(
                output_quat[0],
                output_quat[1],
                output_quat[2],
                output_quat[3],
            )

        current_time = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )
        current_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        forward_speed = 0.0
        yaw_rate = 0.0

        if self.last_pose_xy is not None and self.last_pose_time is not None:
            dt = current_time - self.last_pose_time
            if dt > 1e-6:
                vx_world = (current_xy[0] - self.last_pose_xy[0]) / dt
                vy_world = (current_xy[1] - self.last_pose_xy[1]) / dt
                heading_x = math.cos(self.latest_map_yaw)
                heading_y = math.sin(self.latest_map_yaw)
                forward_speed = vx_world * heading_x + vy_world * heading_y
                if self.imu_enabled and self.use_imu_angular_velocity and self.have_angular_velocity:
                    yaw_rate = self.latest_angular_velocity_z
                elif self.last_output_yaw is not None:
                    yaw_rate = _norm_angle(
                        self.latest_map_yaw - self.last_output_yaw
                    ) / dt

        self.last_pose_xy = current_xy
        self.last_pose_time = current_time
        self.last_output_yaw = self.latest_map_yaw

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.output_frame or msg.header.frame_id
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = float(msg.pose.pose.position.x)
        odom_msg.pose.pose.position.y = float(msg.pose.pose.position.y)
        odom_msg.pose.pose.position.z = float(msg.pose.pose.position.z)
        odom_msg.pose.pose.orientation.x = output_quat[0]
        odom_msg.pose.pose.orientation.y = output_quat[1]
        odom_msg.pose.pose.orientation.z = output_quat[2]
        odom_msg.pose.pose.orientation.w = output_quat[3]
        odom_msg.pose.covariance = list(msg.pose.covariance)

        if len(odom_msg.pose.covariance) >= 36 and imu_fusion_active:
            roll_cov = self.latest_orientation_covariance[0]
            pitch_cov = self.latest_orientation_covariance[4]
            yaw_cov = self.latest_orientation_covariance[8]
            if roll_cov >= 0.0:
                odom_msg.pose.covariance[21] = roll_cov
            if pitch_cov >= 0.0:
                odom_msg.pose.covariance[28] = pitch_cov
            if yaw_cov >= 0.0:
                odom_msg.pose.covariance[35] = yaw_cov

        odom_msg.twist.twist.linear.x = forward_speed
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = yaw_rate
        if (
            len(odom_msg.twist.covariance) >= 36
            and self.imu_enabled
            and self.have_angular_velocity
        ):
            angular_cov = self.latest_angular_velocity_covariance[8]
            if angular_cov >= 0.0:
                odom_msg.twist.covariance[35] = angular_cov

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom_msg.header.stamp
            tf_msg.header.frame_id = odom_msg.header.frame_id
            tf_msg.child_frame_id = odom_msg.child_frame_id
            tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
            tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
            tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanmatchingImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
