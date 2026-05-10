#!/usr/bin/env python3
"""
Visual servo bridge.

Detector publishes /target_position at the detector's effective rate
(~7-15 Hz on this hardware after frame skipping). The gimbal needs a
smooth high-rate command. This node holds the last valid pixel error
and republishes it to /gimbal/cmd/look_at_pixel at servo_hz.

When the target hasn't been seen within target_lost_timeout, this node
stops emitting commands so the gimbal will hold its last attitude
(rather than drift to zero).

Subscribes:
  /target_position    geometry_msgs/PointStamped (normalised -1..1, .z=conf)

Publishes:
  /gimbal/cmd/look_at_pixel    geometry_msgs/PointStamped (normalised -1..1)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class VisualServo(Node):
    def __init__(self):
        super().__init__('visual_servo')

        self.declare_parameter('target_topic', '/target_position')
        self.declare_parameter('command_topic', '/gimbal/cmd/look_at_pixel')
        self.declare_parameter('servo_hz', 50.0)
        self.declare_parameter('target_lost_timeout', 1.5)  # seconds

        self.servo_hz = float(self.get_parameter('servo_hz').value)
        self.lost_timeout_ns = int(self.get_parameter('target_lost_timeout').value * 1e9)

        self.create_subscription(
            PointStamped,
            self.get_parameter('target_topic').value,
            self._on_target,
            10,
        )
        self.cmd_pub = self.create_publisher(
            PointStamped,
            self.get_parameter('command_topic').value,
            10,
        )

        self.last_target = None        # PointStamped
        self.last_target_ns = 0

        self.create_timer(1.0 / self.servo_hz, self._tick)

        self.get_logger().info(
            f"VisualServo up | {self.servo_hz:.0f} Hz | lost_timeout="
            f"{self.lost_timeout_ns/1e9:.1f}s")

    def _on_target(self, msg: PointStamped):
        self.last_target = msg
        self.last_target_ns = self.get_clock().now().nanoseconds

    def _tick(self):
        if self.last_target is None:
            return
        age_ns = self.get_clock().now().nanoseconds - self.last_target_ns
        if age_ns > self.lost_timeout_ns:
            return  # held silent — gimbal holds its attitude
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.last_target.header.frame_id
        out.point.x = self.last_target.point.x
        out.point.y = self.last_target.point.y
        out.point.z = self.last_target.point.z   # forward confidence as-is
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
