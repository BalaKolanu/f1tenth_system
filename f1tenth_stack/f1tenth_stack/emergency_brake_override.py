#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class EmergencyBrakeOverride(Node):
    def __init__(self) -> None:
        super().__init__('emergency_brake_override')

        self.declare_parameter('brake_request_topic', '/safety/emergency_brake')
        self.declare_parameter('motor_speed_topic', 'commands/motor/speed')
        self.declare_parameter('motor_brake_topic', 'commands/motor/brake')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('brake_current', 500.0)
        self.declare_parameter('zero_speed_erpm', 0.0)

        brake_request_topic = str(self.get_parameter('brake_request_topic').value)
        motor_speed_topic = str(self.get_parameter('motor_speed_topic').value)
        motor_brake_topic = str(self.get_parameter('motor_brake_topic').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.brake_current = float(self.get_parameter('brake_current').value)
        self.zero_speed_erpm = float(self.get_parameter('zero_speed_erpm').value)

        self.speed_pub = self.create_publisher(Float64, motor_speed_topic, 10)
        self.brake_pub = self.create_publisher(Float64, motor_brake_topic, 10)
        self.create_subscription(Bool, brake_request_topic, self.handle_brake_request, 10)
        self.create_timer(1.0 / publish_rate_hz, self.handle_timer)

        self.brake_active = False

        self.get_logger().info(
            'Emergency brake override ready. request=%s speed=%s brake=%s current=%.1fA rate=%.1fHz'
            % (
                brake_request_topic,
                motor_speed_topic,
                motor_brake_topic,
                self.brake_current,
                publish_rate_hz,
            )
        )

    def handle_brake_request(self, msg: Bool) -> None:
        was_active = self.brake_active
        self.brake_active = bool(msg.data)
        if self.brake_active != was_active:
            self.get_logger().warn(
                'Emergency brake %s' % ('ENGAGED' if self.brake_active else 'released')
            )

    def handle_timer(self) -> None:
        if not self.brake_active:
            return

        speed_msg = Float64()
        speed_msg.data = self.zero_speed_erpm
        brake_msg = Float64()
        brake_msg.data = self.brake_current
        self.speed_pub.publish(speed_msg)
        self.brake_pub.publish(brake_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmergencyBrakeOverride()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
