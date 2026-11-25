#!/usr/bin/env python3
"""Relay unstamped geometry_msgs/Twist on /cmd_vel to TwistStamped for Jazzy's diff_drive_controller.

Optionally injects a sim-time command latency (latency_s > 0) for domain randomization.
When latency_s == 0 (default), incoming commands are stamped and published immediately
on the original zero-overhead code path.
"""

from collections import deque
from typing import Deque, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


def _copy_twist(msg: Twist) -> Twist:
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.linear.z = msg.linear.z
    out.angular.x = msg.angular.x
    out.angular.y = msg.angular.y
    out.angular.z = msg.angular.z
    return out


class TwistStamper(Node):
    def __init__(self) -> None:
        super().__init__("twist_stamper")
        self.declare_parameter("in_topic", "/cmd_vel")
        self.declare_parameter("out_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("latency_s", 0.0)

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._latency_s = float(self.get_parameter("latency_s").value)
        self._latency_ns = max(0, int(round(self._latency_s * 1e9)))
        self._queue: Deque[Tuple[int, Twist]] = deque()

        self._pub = self.create_publisher(TwistStamped, out_topic, 10)
        self._sub = self.create_subscription(Twist, in_topic, self._on_twist, 10)

        if self._latency_ns > 0:
            # 2 ms sim-time timer matches the MuJoCo physics step for accurate latency release.
            self._timer = self.create_timer(0.002, self._flush_queue)
            self.get_logger().info(
                f"twist_stamper command latency enabled: {self._latency_s * 1000.0:.1f} ms (sim time)"
            )

    def _publish_now(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = msg
        self._pub.publish(stamped)

    def _on_twist(self, msg: Twist) -> None:
        if self._latency_ns <= 0:
            self._publish_now(msg)
            return
        due_ns = self.get_clock().now().nanoseconds + self._latency_ns
        self._queue.append((due_ns, _copy_twist(msg)))

    def _flush_queue(self) -> None:
        if not self._queue:
            return
        now_ns = self.get_clock().now().nanoseconds
        while self._queue and self._queue[0][0] <= now_ns:
            _, msg = self._queue.popleft()
            self._publish_now(msg)


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
