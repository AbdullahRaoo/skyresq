#!/usr/bin/env python3
"""
Visual servo bridge — passthrough.

Forwards each /target_position event to /gimbal/cmd/look_at_pixel
one-to-one. No rate multiplication.

Why pass-through and not "republish at 50 Hz"
  The downstream gimbal_controller treats every pixel command as a
  cumulative delta (target += error * gain). Re-publishing the SAME
  pixel error 50× a second is interpreted as 50 fresh deltas → runaway
  slew. One forward per detection means the cumulative model converges
  cleanly over a few detections.

When the target hasn't been seen for target_lost_timeout seconds, this
node stops emitting commands so the gimbal holds its last attitude
(rather than drifting to zero).

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
        self.declare_parameter('target_lost_timeout', 1.5)  # seconds

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

        self.get_logger().info(
            f"VisualServo (passthrough) up | lost_timeout="
            f"{self.lost_timeout_ns/1e9:.1f}s"
        )

    def _on_target(self, msg: PointStamped):
        # Drop stale events (clock skew or stuck publisher)
        msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg_ns > 0:
            age_ns = self.get_clock().now().nanoseconds - msg_ns
            if age_ns > self.lost_timeout_ns:
                return
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.point.x = msg.point.x
        out.point.y = msg.point.y
        out.point.z = msg.point.z
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
