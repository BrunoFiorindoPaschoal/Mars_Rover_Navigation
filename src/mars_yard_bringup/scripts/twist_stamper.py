#!/usr/bin/env python3
"""Relay unstamped geometry_msgs/Twist on /cmd_vel to TwistStamped for Jazzy's diff_drive_controller."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self) -> None:
        super().__init__("twist_stamper")
        self.declare_parameter("in_topic", "/cmd_vel")
        self.declare_parameter("out_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "base_link")

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self._pub = self.create_publisher(TwistStamped, out_topic, 10)
        self._sub = self.create_subscription(Twist, in_topic, self._on_twist, 10)

    def _on_twist(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = msg
        self._pub.publish(stamped)


def main() -> None:
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
