#!/usr/bin/env python3
"""
payload_servo — drives the rescue payload servo on Pi GPIO via pigpio.

Default wiring: BCM 16 (physical pin 36). Servo runs 500–2500 µs PWM.
The Arduino prototype used 0° = closed/grab, 180° = open/release; we
mirror that mapping.

Subscribes:
  /payload/cmd        std_msgs/String       "open" | "close" | "toggle"

Publishes:
  /payload/state      std_msgs/Bool         True = open, False = closed
                       (latched / TRANSIENT_LOCAL so late subscribers see it)

Parameters:
  gpio_pin            int    BCM pin, default 16
  pwm_closed_us       int    µs for closed/grab (0°), default 500
  pwm_open_us         int    µs for open/release (180°), default 2500
  initial_state       str    "open" | "closed", default "closed"
  detach_after_s      float  release PWM after this many seconds so the
                             servo isn't held under stall. 0 = never.
                             default 1.5

Why lgpio
  pigpio is no longer packaged on Debian Trixie (Raspberry Pi OS 13).
  lgpio talks to the kernel's gpiochip via /dev/gpiochip0, gives us
  hardware-timed servo PWM (`tx_servo`) with no userspace daemon, and is
  already installed by default on current Raspberry Pi OS images.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    import lgpio
    _LGPIO_AVAILABLE = True
except ImportError:
    _LGPIO_AVAILABLE = False


class PayloadServo(Node):
    def __init__(self):
        super().__init__("payload_servo")

        self.declare_parameter("gpio_pin", 16)
        # Match the Arduino Servo library mapping (0° = 544 µs, 180° = 2400 µs).
        # The previous 500/2500 µs range over-drove the servo past its
        # mechanical end-stops, which made it "rotate too much" and grind.
        self.declare_parameter("pwm_closed_us", 544)
        self.declare_parameter("pwm_open_us", 2400)
        self.declare_parameter("initial_state", "closed")
        self.declare_parameter("detach_after_s", 1.5)

        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.pwm_closed = int(self.get_parameter("pwm_closed_us").value)
        self.pwm_open = int(self.get_parameter("pwm_open_us").value)
        self.detach_after_s = float(self.get_parameter("detach_after_s").value)

        self._is_open = (self.get_parameter("initial_state").value == "open")
        self._lock = threading.Lock()
        self._detach_timer = None

        if not _LGPIO_AVAILABLE:
            self.get_logger().error(
                "lgpio not installed. Run: sudo apt install python3-lgpio"
            )
            raise SystemExit(2)

        try:
            self._chip = lgpio.gpiochip_open(0)
            # Claim the line as output so tx_servo owns the toggling.
            lgpio.gpio_claim_output(self._chip, self.gpio_pin, 0)
        except Exception as exc:
            self.get_logger().error(f"lgpio open/claim failed on BCM{self.gpio_pin}: {exc}")
            raise SystemExit(2)

        # Latched state publisher so dashboard / mavlink_bridge get the
        # current state on subscribe.
        state_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(Bool, "/payload/state", state_qos)

        self.create_subscription(String, "/payload/cmd", self._on_cmd, 10)

        # Apply initial state
        self._apply(self._is_open, reason="boot")

        self.get_logger().info(
            f"payload_servo up | pin BCM{self.gpio_pin} | closed={self.pwm_closed}us "
            f"open={self.pwm_open}us | initial={'OPEN' if self._is_open else 'CLOSED'}"
        )

    def _on_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        with self._lock:
            if cmd == "open":
                target = True
            elif cmd in ("close", "closed", "grab"):
                target = False
            elif cmd == "toggle":
                target = not self._is_open
            else:
                self.get_logger().warning(f"ignored payload cmd: {msg.data!r}")
                return
            self._apply(target, reason=f"cmd={cmd}")

    def _apply(self, want_open: bool, reason: str):
        pulse = self.pwm_open if want_open else self.pwm_closed
        try:
            # tx_servo(handle, gpio, pulse_us, freq=50, pulse_cycles=0=continuous)
            lgpio.tx_servo(self._chip, self.gpio_pin, pulse, 50, 0)
        except Exception as exc:
            self.get_logger().error(f"lgpio tx_servo failed: {exc}")
            return
        self._is_open = want_open
        out = Bool()
        out.data = want_open
        self.state_pub.publish(out)
        self.get_logger().info(
            f"payload -> {'OPEN' if want_open else 'CLOSED'} ({pulse}us) [{reason}]"
        )
        # Detach after a moment so the servo isn't held against stall.
        if self.detach_after_s > 0:
            if self._detach_timer is not None:
                self._detach_timer.cancel()
            self._detach_timer = threading.Timer(self.detach_after_s, self._detach)
            self._detach_timer.daemon = True
            self._detach_timer.start()

    def _detach(self):
        try:
            # pulse_us=0 stops the train; line goes idle.
            lgpio.tx_servo(self._chip, self.gpio_pin, 0, 50, 0)
        except Exception:
            pass

    def destroy_node(self):
        try:
            if self._detach_timer is not None:
                self._detach_timer.cancel()
            lgpio.tx_servo(self._chip, self.gpio_pin, 0, 50, 0)
            lgpio.gpiochip_close(self._chip)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PayloadServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
