#!/usr/bin/env python3
"""
Simulated payload servo — SITL only.

The real payload_servo drives Pi GPIO via lgpio and hard-exits on a PC
(no /dev/gpiochip0). This stand-in mirrors `/payload/cmd` →
`/payload/state` with the SAME latched (TRANSIENT_LOCAL) QoS the real
node uses, so sar_orchestrator's DROP → DROP_HOLD → RTL transition runs
exactly as on hardware (it waits to see /payload/state=True after
commanding "open"). Servo timing is not modelled — only the
open/closed contract the state machine depends on.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, String


class SimPayload(Node):
    def __init__(self):
        super().__init__("sim_payload")
        self.declare_parameter("initial_state", "closed")
        self._open = (self.get_parameter("initial_state").value == "open")

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Bool, "/payload/state", qos)
        self.create_subscription(String, "/payload/cmd", self._on_cmd, 10)
        self._publish()
        self.get_logger().info(
            f"sim_payload up | initial={'OPEN' if self._open else 'CLOSED'}")

    def _on_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd == "open":
            self._open = True
        elif cmd in ("close", "closed", "grab"):
            self._open = False
        elif cmd == "toggle":
            self._open = not self._open
        else:
            self.get_logger().warning(f"ignored payload cmd: {msg.data!r}")
            return
        self._publish()
        self.get_logger().info(f"payload -> {'OPEN' if self._open else 'CLOSED'} [{cmd}]")

    def _publish(self):
        b = Bool()
        b.data = self._open
        self._pub.publish(b)


def main(args=None):
    rclpy.init(args=args)
    node = SimPayload()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
