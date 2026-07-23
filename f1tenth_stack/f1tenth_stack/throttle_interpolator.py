#!/usr/bin/env python3

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64


class ThrottleInterpolator(Node):
    def __init__(self) -> None:
        super().__init__('throttle_interpolator')

        self.declare_parameter('input_topic', 'commands/motor/unsmoothed_speed')
        self.declare_parameter('output_topic', 'commands/motor/speed')
        self.declare_parameter('publish_hz', 100.0)
        self.declare_parameter('launch_enabled', True)
        self.declare_parameter('launch_autonomy_only', True)
        self.declare_parameter('autonomy_enabled_topic', '/safety/autonomy_enabled')
        self.declare_parameter('launch_duration_sec', 1.0)
        self.declare_parameter('speed_to_erpm_gain', 3278.071192)
        self.declare_parameter('launch_floor_erpm', 0.0)
        self.declare_parameter('launch_start_speed_mps', 0.05)
        self.declare_parameter('launch_reset_speed_mps', 0.02)
        self.declare_parameter(
            'launch_profile_time_points_sec',
            [0.0, 0.05, 0.12, 0.20, 0.32, 0.48, 0.66, 0.82, 1.0],
        )
        self.declare_parameter(
            'launch_profile_scale_points',
            [0.0, 0.01, 0.04, 0.10, 0.22, 0.42, 0.68, 0.88, 1.0],
        )
        self.declare_parameter('max_acceleration_mps2', 0.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self.launch_enabled = bool(self.get_parameter('launch_enabled').value)
        self.launch_autonomy_only = bool(self.get_parameter('launch_autonomy_only').value)
        self.autonomy_enabled_topic = str(self.get_parameter('autonomy_enabled_topic').value)
        self.launch_duration_sec = max(1e-3, float(self.get_parameter('launch_duration_sec').value))
        self.speed_to_erpm_gain = float(self.get_parameter('speed_to_erpm_gain').value)
        self.launch_floor_erpm = max(0.0, float(self.get_parameter('launch_floor_erpm').value))
        self.launch_start_speed_mps = max(0.0, float(self.get_parameter('launch_start_speed_mps').value))
        self.launch_reset_speed_mps = max(0.0, float(self.get_parameter('launch_reset_speed_mps').value))
        self.max_acceleration_mps2 = max(0.0, float(self.get_parameter('max_acceleration_mps2').value))
        self._profile_t, self._profile_gain = self._load_profile()
        self.launch_floor_speed_mps = 0.0
        if abs(self.speed_to_erpm_gain) > 1e-6:
            self.launch_floor_speed_mps = self.launch_floor_erpm / abs(self.speed_to_erpm_gain)

        self._have_input = False
        self._last_input_speed = 0.0
        self._target_speed = 0.0
        self._current_output_speed = 0.0
        self._launch_active = False
        self._launch_start_monotonic = 0.0
        self._last_publish_monotonic = time.monotonic()
        self._autonomy_enabled = not self.launch_autonomy_only

        self._pub = self.create_publisher(Float64, self.output_topic, 10)
        self.create_subscription(Float64, self.input_topic, self.input_cb, 10)
        self.create_subscription(Bool, self.autonomy_enabled_topic, self.autonomy_cb, 10)
        self._timer = self.create_timer(1.0 / self.publish_hz, self.timer_cb)

        self.get_logger().info(
            'Throttle interpolator ready. input=%s output=%s enabled=%s autonomy_only=%s launch=%.3fs floor_erpm=%.0f profile_points=%d'
            % (
                self.input_topic,
                self.output_topic,
                str(self.launch_enabled).lower(),
                str(self.launch_autonomy_only).lower(),
                self.launch_duration_sec,
                self.launch_floor_erpm,
                len(self._profile_t),
            )
        )

    def _load_profile(self):
        raw_t = [float(v) for v in self.get_parameter('launch_profile_time_points_sec').value]
        raw_gain = [float(v) for v in self.get_parameter('launch_profile_scale_points').value]
        if len(raw_t) != len(raw_gain) or len(raw_t) < 2:
            self.get_logger().warning(
                'Invalid launch profile arrays; falling back to a linear 0..1 profile.'
            )
            return [0.0, 1.0], [0.0, 1.0]

        pairs = sorted(zip(raw_t, raw_gain), key=lambda pair: pair[0])
        cleaned_t: List[float] = []
        cleaned_gain: List[float] = []
        last_t = -math.inf
        for t, gain in pairs:
            if not math.isfinite(t) or not math.isfinite(gain):
                continue
            t = min(self.launch_duration_sec, max(0.0, t))
            gain = max(0.0, gain)
            if t <= last_t:
                t = min(self.launch_duration_sec, last_t + 1e-6)
            cleaned_t.append(t)
            cleaned_gain.append(gain)
            last_t = t

        if len(cleaned_t) < 2:
            self.get_logger().warning(
                'Launch profile sanitization removed too many points; using linear fallback.'
            )
            return [0.0, 1.0], [0.0, 1.0]

        if cleaned_t[0] > 0.0:
            cleaned_t.insert(0, 0.0)
            cleaned_gain.insert(0, 0.0)
        if cleaned_t[-1] < self.launch_duration_sec:
            cleaned_t.append(self.launch_duration_sec)
            cleaned_gain.append(1.0)
        return cleaned_t, cleaned_gain

    def _interp_profile(self, elapsed_sec: float) -> float:
        if elapsed_sec <= self._profile_t[0]:
            return self._profile_gain[0]
        if elapsed_sec >= self._profile_t[-1]:
            return self._profile_gain[-1]

        for idx in range(1, len(self._profile_t)):
            t0 = self._profile_t[idx - 1]
            t1 = self._profile_t[idx]
            if elapsed_sec <= t1:
                g0 = self._profile_gain[idx - 1]
                g1 = self._profile_gain[idx]
                if t1 <= t0:
                    return g1
                blend = (elapsed_sec - t0) / (t1 - t0)
                return g0 + blend * (g1 - g0)
        return self._profile_gain[-1]

    def _reset_launch(self) -> None:
        self._launch_active = False
        self._launch_start_monotonic = 0.0

    def autonomy_cb(self, msg: Bool) -> None:
        self._autonomy_enabled = bool(msg.data)
        if not self._autonomy_enabled:
            self._reset_launch()

    def _publish(self, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        self._current_output_speed = msg.data
        self._pub.publish(msg)
        self._last_publish_monotonic = time.monotonic()

    def _should_start_launch(self, target_speed: float) -> bool:
        if not self.launch_enabled:
            return False
        if target_speed <= self.launch_start_speed_mps:
            return False
        if self._current_output_speed > self.launch_reset_speed_mps:
            return False
        return self._last_input_speed <= self.launch_reset_speed_mps

    def input_cb(self, msg: Float64) -> None:
        target = float(msg.data)
        self._have_input = True
        self._last_input_speed = self._target_speed
        self._target_speed = target

        if self.launch_autonomy_only and not self._autonomy_enabled:
            self._reset_launch()
            self._publish(target)
            return

        if target <= self.launch_reset_speed_mps:
            self._reset_launch()
            self._publish(target)
            return

        if target < 0.0:
            self._reset_launch()
            self._publish(target)
            return

        if self._launch_active and target < self._current_output_speed - 1e-6:
            self._reset_launch()
            self._publish(target)
            return

        if self._should_start_launch(target):
            self._launch_active = True
            self._launch_start_monotonic = time.monotonic()
            self._publish(self._evaluate_launch_output(time.monotonic()))
            return

        if not self._launch_active:
            self._publish(target)

    def _evaluate_launch_output(self, now_monotonic: float) -> float:
        elapsed = max(0.0, now_monotonic - self._launch_start_monotonic)
        launch_fraction = max(0.0, self._interp_profile(min(elapsed, self.launch_duration_sec)))
        shaped = min(self._target_speed, self._target_speed * launch_fraction)
        if self._target_speed > 0.0 and self.launch_floor_speed_mps > 0.0:
            shaped = min(self._target_speed, max(shaped, self.launch_floor_speed_mps))

        if self.max_acceleration_mps2 > 0.0:
            dt = max(0.0, now_monotonic - self._last_publish_monotonic)
            shaped = min(shaped, self._current_output_speed + self.max_acceleration_mps2 * dt)

        if elapsed >= self.launch_duration_sec or launch_fraction >= 0.999:
            self._reset_launch()
            return self._target_speed
        return shaped

    def timer_cb(self) -> None:
        if not self._have_input or not self._launch_active:
            return
        if self.launch_autonomy_only and not self._autonomy_enabled:
            self._reset_launch()
            self._publish(self._target_speed)
            return
        if self._target_speed < 0.0:
            self._reset_launch()
            self._publish(self._target_speed)
            return
        if self._target_speed <= self.launch_reset_speed_mps:
            self._reset_launch()
            self._publish(self._target_speed)
            return
        self._publish(self._evaluate_launch_output(time.monotonic()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrottleInterpolator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
