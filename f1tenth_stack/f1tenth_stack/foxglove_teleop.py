#!/usr/bin/env python3

"""Convert Foxglove's Twist teleop messages to F1TENTH drive commands."""

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class FoxgloveTeleop(Node):
    def __init__(self):
        super().__init__('foxglove_teleop')
        self.declare_parameter('input_topic', '/foxglove/teleop')
        self.declare_parameter('output_topic', 'teleop')
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('steering_scale', 1.0)
        self.declare_parameter('max_speed_mps', 6.0)
        self.declare_parameter('max_steering_angle_rad', 0.45)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.steering_scale = float(self.get_parameter('steering_scale').value)
        self.max_speed = abs(float(self.get_parameter('max_speed_mps').value))
        self.max_steering = abs(
            float(self.get_parameter('max_steering_angle_rad').value)
        )

        self.publisher = self.create_publisher(
            AckermannDriveStamped, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Twist, input_topic, self.twist_callback, 10
        )
        self.get_logger().info(
            'Foxglove teleop: %s (Twist) -> %s (AckermannDriveStamped)' %
            (input_topic, output_topic)
        )

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def twist_callback(self, msg):
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.drive.speed = self.clamp(
            msg.linear.x * self.speed_scale, self.max_speed
        )
        # Foxglove angular.z is treated as a direct steering-angle command.
        output.drive.steering_angle = self.clamp(
            msg.angular.z * self.steering_scale, self.max_steering
        )
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
