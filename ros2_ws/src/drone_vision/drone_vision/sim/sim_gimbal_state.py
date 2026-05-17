#!/usr/bin/env python3
"""
Simulated gimbal-state publisher — SITL/Gazebo only.

The deployed gimbal_controller talks to the XF Z-1 Mini over TCP and
publishes the real gimbal orientation on `/gimbal/state`. In Gazebo we
fix the iris's 3-axis gimbal at nadir (pitch=-90° in the body frame)
via a one-shot Gazebo topic command at startup; this node mirrors that
into ROS as the static gimbal state geo_localiser needs.

Publishes:
  /gimbal/state    geometry_msgs/Vector3Stamped   x=roll y=pitch z=yaw (deg)

Parameters:
  pitch_deg        default -90.0   (nadir; frames.py expects negative = down)
  yaw_deg          default   0.0
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


class SimGimbalState(Node):
    def __init__(self):
        super().__init__("sim_gimbal_state")
        self.declare_parameter("pitch_deg", -90.0)
        self.declare_parameter("yaw_deg", 0.0)
        self._pitch = float(self.get_parameter("pitch_deg").value)
        self._yaw = float(self.get_parameter("yaw_deg").value)
        self._pub = self.create_publisher(Vector3Stamped, "/gimbal/state", 10)
        self.create_timer(0.2, self._tick)   # 5 Hz
        self.get_logger().info(
            f"sim_gimbal_state up | pitch={self._pitch}° yaw={self._yaw}°")

    def _tick(self):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.vector.x = 0.0
        m.vector.y = self._pitch
        m.vector.z = self._yaw
        self._pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = SimGimbalState()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
