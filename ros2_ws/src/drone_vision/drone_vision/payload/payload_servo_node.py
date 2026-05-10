#!/usr/bin/env python3
"""
Payload servo — real (pigpio) backend.

Runs on the Raspberry Pi. Drives a servo via hardware-PWM on a configurable
GPIO pin through the pigpio daemon (`pigpiod` must be running — provided by
the system service installed in PR-6).

Same ROS surface as the sim stub (PR-2b's payload_servo_sim):
  - Service /payload/drop  drone_msgs/srv/DropPayload
  - Topic   /payload/state std_msgs/String  (latched)

The interlocks are identical to the sim — the sim is the contract, the real
node is the implementation. Mission code never has to know which is running.

Bench test before flight (Pi only):
  $ sudo systemctl start pigpiod
  $ ros2 run drone_vision payload_servo
  $ ros2 service call /payload/drop drone_msgs/srv/DropPayload "{arm_drop: true}"
  Expect servo to swing from closed to open and back after open_hold_secs.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus

from drone_msgs.msg import TargetWorld
from drone_msgs.srv import DropPayload


# pigpio is not importable on the dev PC — defer import so this file can be
# loaded for inspection, but fail loudly at startup if the backend isn't 'sim'.
try:
    import pigpio  # type: ignore
    _PIGPIO_AVAILABLE = True
except ImportError:
    pigpio = None
    _PIGPIO_AVAILABLE = False


class PayloadServoNode(Node):
    STATE_ARMED   = 'armed'
    STATE_READY   = 'ready'
    STATE_DROPPED = 'dropped'
    STATE_FAULT   = 'fault'

    def __init__(self):
        super().__init__('payload_servo')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('backend', 'pigpio')   # 'pigpio' | 'sim'
        self.declare_parameter('gpio_pin', 18)        # BCM, hardware-PWM-capable on Pi 4
        self.declare_parameter('pwm_us_closed', 1100)
        self.declare_parameter('pwm_us_open', 1900)
        self.declare_parameter('open_hold_secs', 3.0)
        self.declare_parameter('min_drop_alt_m', 1.5)
        self.declare_parameter('max_drop_alt_m', 10.0)
        self.declare_parameter('target_freshness_secs', 1.5)
        self.declare_parameter('require_armed_offboard', True)
        self.declare_parameter('require_battery_above', 0.20)
        self.declare_parameter('state_publish_hz', 1.0)

        backend = self.get_parameter('backend').value
        self.gpio_pin = int(self.get_parameter('gpio_pin').value)
        self.pwm_closed = int(self.get_parameter('pwm_us_closed').value)
        self.pwm_open   = int(self.get_parameter('pwm_us_open').value)
        self.open_hold  = float(self.get_parameter('open_hold_secs').value)
        self.min_alt = float(self.get_parameter('min_drop_alt_m').value)
        self.max_alt = float(self.get_parameter('max_drop_alt_m').value)
        self.target_freshness = float(self.get_parameter('target_freshness_secs').value)
        self.require_armed = bool(self.get_parameter('require_armed_offboard').value)
        self.require_batt = float(self.get_parameter('require_battery_above').value)
        state_hz = float(self.get_parameter('state_publish_hz').value)

        if backend != 'pigpio':
            raise RuntimeError(
                f"payload_servo_node only supports backend='pigpio'; got '{backend}'. "
                "For sim launches use payload_servo_sim instead.")
        if not _PIGPIO_AVAILABLE:
            raise RuntimeError(
                "pigpio Python binding is not installed. On the Pi: `sudo apt install pigpio python3-pigpio`")

        # ── pigpio connection ─────────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "Cannot connect to pigpiod. Is the daemon running? `sudo systemctl start pigpiod`")
        # Initialise to closed
        self.pi.set_servo_pulsewidth(self.gpio_pin, self.pwm_closed)

        # ── Sensor state ──────────────────────────────────────────────
        self.agl_m = None
        self.armed = False
        self.offboard = False
        self.battery_remaining = 1.0
        self.last_target_world_t = 0.0
        self.last_target_ned = None

        # ── State + publishers / subscribers ───────────────────────────
        self.state = self.STATE_ARMED
        self._drop_lock = threading.Lock()
        self._dropped = False    # one-shot mission flag

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.state_pub = self.create_publisher(String, '/payload/state', latched_qos)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self._on_local_pos, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._on_status, px4_qos)
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status',
                                 self._on_battery, px4_qos)
        self.create_subscription(TargetWorld, '/target/world',
                                 self._on_target_world, 10)

        self.create_service(DropPayload, '/payload/drop', self._handle_drop)
        self.create_timer(1.0 / state_hz, self._publish_state)

        self._publish_state()
        self.get_logger().info(
            f"PayloadServoNode (pigpio) up | GPIO {self.gpio_pin} | "
            f"closed={self.pwm_closed}us open={self.pwm_open}us | "
            f"alt range [{self.min_alt}, {self.max_alt}] m")

    # ── Sensor callbacks ───────────────────────────────────────────────

    def _on_local_pos(self, msg: VehicleLocalPosition):
        self.agl_m = -msg.z

    def _on_status(self, msg: VehicleStatus):
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _on_battery(self, msg: BatteryStatus):
        if 0.0 <= msg.remaining <= 1.0:
            self.battery_remaining = float(msg.remaining)

    def _on_target_world(self, msg: TargetWorld):
        self.last_target_world_t = time.monotonic()
        self.last_target_ned = (msg.position_ned.point.x,
                                msg.position_ned.point.y,
                                msg.position_ned.point.z)

    # ── Drop service ───────────────────────────────────────────────────

    def _handle_drop(self, request, response):
        with self._drop_lock:
            ok, reason = self._check_interlocks(request.arm_drop)
            if not ok:
                response.ok = False
                response.reason = reason
                self.get_logger().warn(f"Drop refused: {reason}")
                return response

            # Optional pre-drop delay — run on a thread so we don't block
            # the rclpy executor for hundreds of ms.
            delay = max(0, int(request.delay_ms)) / 1000.0
            if delay > 0:
                self.get_logger().info(f"Dropping in {delay:.2f}s")
                threading.Timer(delay, self._do_drop).start()
            else:
                self._do_drop()
            response.ok = True
            response.reason = ""
            return response

    def _do_drop(self):
        with self._drop_lock:
            if self._dropped:
                return
            try:
                self.pi.set_servo_pulsewidth(self.gpio_pin, self.pwm_open)
                self.get_logger().info(
                    f"PAYLOAD DROPPED | servo→{self.pwm_open}us | "
                    f"AGL {self.agl_m:.1f} m | battery {self.battery_remaining:.0%}")
                self._dropped = True
                self._set_state(self.STATE_DROPPED)
            except Exception as e:
                self.get_logger().error(f"pigpio write failed: {e}")
                self._set_state(self.STATE_FAULT)
                return
            # Schedule a re-close after open_hold_secs (one-shot)
            threading.Timer(self.open_hold, self._reclose).start()

    def _reclose(self):
        try:
            self.pi.set_servo_pulsewidth(self.gpio_pin, self.pwm_closed)
            self.get_logger().info(f"Servo re-closed → {self.pwm_closed}us")
        except Exception as e:
            self.get_logger().warn(f"reclose failed: {e}")

    def _check_interlocks(self, arm_drop):
        if not arm_drop:
            return False, "request.arm_drop must be true"
        if self.agl_m is None:
            return False, "no altitude reading yet"
        if self.agl_m < self.min_alt:
            return False, f"AGL {self.agl_m:.1f} m below min_drop_alt {self.min_alt}"
        if self.agl_m > self.max_alt:
            return False, f"AGL {self.agl_m:.1f} m above max_drop_alt {self.max_alt}"
        if self.require_armed and not (self.armed and self.offboard):
            return False, f"vehicle not armed in OFFBOARD (armed={self.armed}, offboard={self.offboard})"
        if self.battery_remaining < self.require_batt:
            return False, f"battery {self.battery_remaining:.0%} below {self.require_batt:.0%}"
        age = time.monotonic() - self.last_target_world_t
        if self.last_target_world_t == 0.0 or age > self.target_freshness:
            return False, f"no fresh /target/world (age={age:.1f}s)"
        if self._dropped:
            return False, "already dropped this mission"
        return True, ""

    def _set_state(self, state):
        self.state = state
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def destroy_node(self):
        # Safety: leave the servo in 'closed' before exiting.
        try:
            if hasattr(self, 'pi') and self.pi.connected:
                self.pi.set_servo_pulsewidth(self.gpio_pin, self.pwm_closed)
                self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PayloadServoNode()
    except RuntimeError as e:
        # Friendly bail-out when run on a non-Pi (e.g. dev box)
        print(f"[payload_servo_node] startup failed: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
