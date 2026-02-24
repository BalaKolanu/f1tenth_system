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
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import yaml


class LinearOdomCalibrator(Node):

    def __init__(self):
        super().__init__('linear_odom_calibrator')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reference_topic', '/scanmatching_odom/pose')
        self.declare_parameter('sampling_frequency', 20.0)
        self.declare_parameter('max_yaw_rate', 0.10)
        self.declare_parameter('max_pair_jump', 0.25)
        self.declare_parameter('min_pair_step', 0.002)
        self.declare_parameter('min_reference_distance', 2.0)
        self.declare_parameter('vesc_config_path', '')
        self.declare_parameter('speed_to_erpm_gain', 0.0)

        odom_topic = self.get_parameter('odom_topic').value
        reference_topic = self.get_parameter('reference_topic').value
        sample_hz = float(self.get_parameter('sampling_frequency').value)
        if sample_hz <= 0.0:
            sample_hz = 20.0

        self.max_yaw_rate = abs(float(self.get_parameter('max_yaw_rate').value))
        self.max_pair_jump = float(self.get_parameter('max_pair_jump').value)
        self.min_pair_step = float(self.get_parameter('min_pair_step').value)
        self.min_reference_distance = float(
            self.get_parameter('min_reference_distance').value
        )

        self.latest_odom_xy = None
        self.latest_odom_yaw_rate = 0.0
        self.latest_reference_xy = None

        self.prev_odom_xy = None
        self.prev_reference_xy = None

        self.total_odom_distance = 0.0
        self.total_reference_distance = 0.0
        self.accepted_samples = 0
        self.report_printed = False

        self.speed_to_erpm_gain = self._load_speed_to_erpm_gain()

        self.create_subscription(Odometry, odom_topic, self.handle_odom, 30)
        self.create_subscription(
            PoseWithCovarianceStamped,
            reference_topic,
            self.handle_reference,
            30,
        )
        self.timer = self.create_timer(1.0 / sample_hz, self.handle_timer)

        rclpy.get_default_context().on_shutdown(self.print_results)
        self.get_logger().info(
            'Linear odom calibration started. Drive mostly straight while keeping '
            'steering centered. Results are printed on shutdown.'
        )

    def _load_speed_to_erpm_gain(self):
        gain_param = float(self.get_parameter('speed_to_erpm_gain').value)
        if gain_param > 0.0:
            return gain_param

        config_path = self.get_parameter('vesc_config_path').value
        if not config_path:
            return None

        try:
            with open(config_path, 'r', encoding='utf-8') as yaml_file:
                config = yaml.safe_load(yaml_file) or {}
            ros_params = config.get('/**', {}).get('ros__parameters', {})
            gain = ros_params.get('speed_to_erpm_gain', None)
            if gain is None:
                return None
            return float(gain)
        except Exception as exc:
            self.get_logger().warning(
                'Could not read speed_to_erpm_gain from vesc config: %s' % exc
            )
            return None

    def handle_odom(self, msg):
        self.latest_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.latest_odom_yaw_rate = msg.twist.twist.angular.z

    def handle_reference(self, msg):
        self.latest_reference_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def handle_timer(self):
        if self.latest_odom_xy is None or self.latest_reference_xy is None:
            return

        if abs(self.latest_odom_yaw_rate) > self.max_yaw_rate:
            # Ignore turning sections for linear-only calibration.
            self.prev_odom_xy = None
            self.prev_reference_xy = None
            return

        if self.prev_odom_xy is None or self.prev_reference_xy is None:
            self.prev_odom_xy = self.latest_odom_xy
            self.prev_reference_xy = self.latest_reference_xy
            return

        odom_step = self._distance(self.latest_odom_xy, self.prev_odom_xy)
        reference_step = self._distance(self.latest_reference_xy, self.prev_reference_xy)

        self.prev_odom_xy = self.latest_odom_xy
        self.prev_reference_xy = self.latest_reference_xy

        if odom_step > self.max_pair_jump or reference_step > self.max_pair_jump:
            return

        if odom_step < self.min_pair_step and reference_step < self.min_pair_step:
            return

        self.total_odom_distance += odom_step
        self.total_reference_distance += reference_step
        self.accepted_samples += 1

    @staticmethod
    def _distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def print_results(self):
        if self.report_printed:
            return
        self.report_printed = True

        self.get_logger().info('========== LINEAR ODOM CALIBRATION RESULT ==========')
        self.get_logger().info('accepted_samples: %d' % self.accepted_samples)
        self.get_logger().info(
            'odom_distance_m: %.6f' % self.total_odom_distance
        )
        self.get_logger().info(
            'reference_distance_m: %.6f' % self.total_reference_distance
        )

        if self.total_odom_distance <= 1e-6:
            self.get_logger().warning(
                'Not enough odom motion captured. Repeat calibration with longer '
                'straight driving.'
            )
            self.get_logger().info('====================================================')
            return

        linear_scale = self.total_reference_distance / self.total_odom_distance
        self.get_logger().info('linear_scale_ref_over_odom: %.6f' % linear_scale)

        if self.total_reference_distance < self.min_reference_distance:
            self.get_logger().warning(
                'Reference distance is very short. Recommended to collect at least '
                '%.2f m for stable estimates.' % self.min_reference_distance
            )

        if self.speed_to_erpm_gain is not None and linear_scale > 1e-6:
            recommended_gain = self.speed_to_erpm_gain / linear_scale
            self.get_logger().info(
                'current_speed_to_erpm_gain: %.6f' % self.speed_to_erpm_gain
            )
            self.get_logger().info(
                'recommended_speed_to_erpm_gain: %.6f' % recommended_gain
            )
        else:
            self.get_logger().info(
                'recommended_speed_to_erpm_gain: unavailable '
                '(set speed_to_erpm_gain param or vesc_config_path)'
            )

        self.get_logger().info('====================================================')


def main():
    rclpy.init()
    node = LinearOdomCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
