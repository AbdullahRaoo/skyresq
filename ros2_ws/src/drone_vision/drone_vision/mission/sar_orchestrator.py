#!/usr/bin/env python3
"""
SAR Orchestrator — state machine that drives the autonomous demo mission.

States
------
IDLE              waiting for operator engagement
SEARCH            executing uploaded mission (AUTO mode) — passively watching detector
DETECTION_HOLD    fresh person detection; holding for hold_time_s before committing
APPROACH          flying to a drop point 4 m from the survivor on the drone-side
DROP              servo OPEN issued, waiting for it to settle
DROP_HOLD         payload released, waiting drop_hold_s before closing servo
RTL               return-to-launch issued, waiting for arrival home
DONE              mission complete

Safety invariants
-----------------
- Orchestrator starts in IDLE and refuses to leave it unless /mission/enable=True.
- Any autonomy step refuses if /vehicle/mode reports anything other than GUIDED
  (for setpoint phases) or AUTO (for search). If pilot flips TX switch out of
  GUIDED, ArduPilot reverts mode automatically and we abort to IDLE.
- /mission/enable=False at any time returns to IDLE (operator kill-switch).

Subscribes
----------
  /target/world           drone_msgs/TargetWorld   geo-lock'd survivor
  /vehicle/gps            sensor_msgs/NavSatFix    drone GPS (lat/lon)
  /vehicle/armed          std_msgs/Bool            FC arm state
  /vehicle/mode           std_msgs/String          FC flight mode
  /mission/enable         std_msgs/Bool            operator engagement toggle
  /payload/state          std_msgs/Bool            servo open/closed feedback

Publishes
---------
  /mission/state          drone_msgs/MissionState  current state + telemetry
  /payload/cmd            std_msgs/String          "open" | "close"
  /mission/fly_to         sensor_msgs/NavSatFix    requested setpoint (intent;
                                                   actual FC command sent by
                                                   mavlink_bridge in a later
                                                   iteration)
  /mission/cmd_rtl        std_msgs/Empty           request RTL (intent; same)

This iteration: state-machine + drop-point geometry only. FC command sending
arrives in the next change to mavlink_bridge.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import NavSatFix
from drone_msgs.msg import MissionState, TargetWorld


EARTH_R = 6_371_000.0


def haversine_m(lat1, lon1, lat2, lon2):
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = (math.sin(dLat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dLon / 2) ** 2)
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing from (lat1,lon1) to (lat2,lon2), degrees [0..360)."""
    dLon = math.radians(lon2 - lon1)
    la1 = math.radians(lat1)
    la2 = math.radians(lat2)
    y = math.sin(dLon) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dLon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def offset_from(lat, lon, bearing_d, dist_m):
    """Walk dist_m along bearing from (lat,lon), return new lat/lon."""
    b = math.radians(bearing_d)
    la1 = math.radians(lat)
    lo1 = math.radians(lon)
    d_R = dist_m / EARTH_R
    la2 = math.asin(
        math.sin(la1) * math.cos(d_R) +
        math.cos(la1) * math.sin(d_R) * math.cos(b)
    )
    lo2 = lo1 + math.atan2(
        math.sin(b) * math.sin(d_R) * math.cos(la1),
        math.cos(d_R) - math.sin(la1) * math.sin(la2),
    )
    return math.degrees(la2), math.degrees(lo2)


def compute_drop_point(drone_lat, drone_lon, surv_lat, surv_lon, offset_m=4.0):
    """Drop point: `offset_m` from survivor, on the side facing the drone.

    Avoids overflying the person — drop is short of them along the
    drone→survivor line. If drone and survivor coincide, returns survivor.
    """
    dist = haversine_m(drone_lat, drone_lon, surv_lat, surv_lon)
    if dist < 0.1:
        return surv_lat, surv_lon
    b_surv_to_drone = bearing_deg(surv_lat, surv_lon, drone_lat, drone_lon)
    return offset_from(surv_lat, surv_lon, b_surv_to_drone, offset_m)


STATE_IDLE = "IDLE"
STATE_SEARCH = "SEARCH"
STATE_DETECTION_HOLD = "DETECTION_HOLD"
STATE_APPROACH = "APPROACH"
STATE_DROP = "DROP"
STATE_DROP_HOLD = "DROP_HOLD"
STATE_RTL = "RTL"
STATE_DONE = "DONE"


class SarOrchestrator(Node):
    def __init__(self):
        super().__init__("sar_orchestrator")

        self.declare_parameter("hold_time_s",       1.0)
        self.declare_parameter("approach_tol_m",    2.5)
        self.declare_parameter("drop_offset_m",     4.0)
        self.declare_parameter("drop_hold_s",       3.0)
        self.declare_parameter("min_confidence",    0.45)
        self.declare_parameter("home_tol_m",        4.0)
        self.declare_parameter("tick_hz",           5.0)
        # Grace period before we abort an APPROACH/DROP because the FC
        # briefly reported a non-GUIDED mode. Without this the orchestrator
        # is too twitchy — any single-tick mode read from a hiccupping link
        # kills the mission.
        self.declare_parameter("mode_abort_grace_s", 2.0)
        # Hard timeout for RTL — if we don't see home within this long,
        # declare DONE so the state machine doesn't hang forever (e.g. on
        # the bench with no GPS).
        self.declare_parameter("rtl_max_s",         180.0)

        self._hold_time_s     = float(self.get_parameter("hold_time_s").value)
        self._approach_tol_m  = float(self.get_parameter("approach_tol_m").value)
        self._drop_offset_m   = float(self.get_parameter("drop_offset_m").value)
        self._drop_hold_s     = float(self.get_parameter("drop_hold_s").value)
        self._min_confidence  = float(self.get_parameter("min_confidence").value)
        self._home_tol_m      = float(self.get_parameter("home_tol_m").value)
        self._mode_abort_grace_s = float(self.get_parameter("mode_abort_grace_s").value)
        self._rtl_max_s       = float(self.get_parameter("rtl_max_s").value)
        tick_hz               = float(self.get_parameter("tick_hz").value)

        # Track when we first saw a wrong mode so we can apply grace period
        self._wrong_mode_since: float = 0.0

        # ── State ─────────────────────────────────────────────
        self._state = STATE_IDLE
        self._enabled = False
        self._armed = False
        self._mode = "UNKNOWN"
        self._gimbal_healthy = False
        self._gimbal_health_t = 0.0
        self._drone_lat = None
        self._drone_lon = None
        self._home_lat = None
        self._home_lon = None
        self._last_target: TargetWorld | None = None
        self._last_target_t = 0.0
        self._payload_open = False

        self._hold_start_t = 0.0
        self._drop_point: tuple[float, float] | None = None
        self._drop_open_t = 0.0
        self._state_entered_t = time.monotonic()

        # ── Subs ──────────────────────────────────────────────
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(TargetWorld, "/target/world", self._on_target, 10)
        self.create_subscription(NavSatFix, "/vehicle/gps", self._on_gps, 10)
        self.create_subscription(NavSatFix, "/vehicle/home", self._on_home, latched)
        self.create_subscription(Bool, "/vehicle/armed", self._on_armed, 10)
        self.create_subscription(String, "/vehicle/mode", self._on_mode, 10)
        self.create_subscription(Bool, "/mission/enable", self._on_enable, latched)
        self.create_subscription(Bool, "/payload/state", self._on_payload, latched)
        self.create_subscription(Bool, "/gimbal/health", self._on_gimbal_health, 10)

        # ── Pubs ──────────────────────────────────────────────
        self._pub_state = self.create_publisher(MissionState, "/mission/state", 10)
        self._pub_payload_cmd = self.create_publisher(String, "/payload/cmd", 10)
        self._pub_flyto = self.create_publisher(NavSatFix, "/mission/fly_to", 10)
        self._pub_rtl = self.create_publisher(Empty, "/mission/cmd_rtl", 10)
        self._pub_set_mode = self.create_publisher(String, "/mission/set_mode", 10)

        self.create_timer(1.0 / max(tick_hz, 1.0), self._tick)

        self.get_logger().info(
            f"sar_orchestrator up | hold={self._hold_time_s}s "
            f"drop_offset={self._drop_offset_m}m drop_hold={self._drop_hold_s}s "
            f"min_conf={self._min_confidence}"
        )

    # ── Subscriber callbacks ─────────────────────────────────

    def _on_target(self, msg: TargetWorld):
        self._last_target = msg
        self._last_target_t = time.monotonic()

    def _on_gps(self, msg: NavSatFix):
        if msg.latitude != 0.0 or msg.longitude != 0.0:
            self._drone_lat = msg.latitude
            self._drone_lon = msg.longitude

    def _on_home(self, msg: NavSatFix):
        if msg.latitude != 0.0 or msg.longitude != 0.0:
            self._home_lat = msg.latitude
            self._home_lon = msg.longitude

    def _on_armed(self, msg: Bool):
        self._armed = bool(msg.data)

    def _on_mode(self, msg: String):
        self._mode = (msg.data or "UNKNOWN").upper()

    def _on_gimbal_health(self, msg: Bool):
        self._gimbal_healthy = bool(msg.data)
        self._gimbal_health_t = time.monotonic()

    def _on_enable(self, msg: Bool):
        was = self._enabled
        self._enabled = bool(msg.data)
        if was and not self._enabled:
            self._enter(STATE_IDLE, reason="enable=false")
        elif not was and self._enabled:
            self.get_logger().info("operator engaged SAR autonomy")

    def _on_payload(self, msg: Bool):
        self._payload_open = bool(msg.data)

    # ── State machine tick ───────────────────────────────────

    def _tick(self):
        # Hard kill-switch: any time we lose engagement, drop to IDLE.
        if not self._enabled and self._state != STATE_IDLE:
            self._enter(STATE_IDLE, reason="operator disengaged")
            return

        # Continuously check mode for setpoint-issuing states. ArduPilot
        # auto-reverts mode when the TX switch moves, so we use that as
        # our pilot-takeover detector. Grace period prevents single-tick
        # mode hiccups (link blip, race between heartbeats) from killing
        # the mission.
        now_m = time.monotonic()
        if self._state in (STATE_APPROACH, STATE_DROP, STATE_DROP_HOLD):
            if self._mode != "GUIDED":
                if self._wrong_mode_since == 0.0:
                    self._wrong_mode_since = now_m
                elif (now_m - self._wrong_mode_since) >= self._mode_abort_grace_s:
                    self._enter(STATE_IDLE,
                                reason=f"pilot took over (mode={self._mode} for {now_m - self._wrong_mode_since:.1f}s)")
                    self._wrong_mode_since = 0.0
                    return
            else:
                self._wrong_mode_since = 0.0
        if self._state == STATE_RTL and self._mode != "RTL":
            if self._wrong_mode_since == 0.0:
                self._wrong_mode_since = now_m
            elif (now_m - self._wrong_mode_since) >= self._mode_abort_grace_s:
                self._enter(STATE_IDLE, reason="RTL cancelled by pilot")
                self._wrong_mode_since = 0.0
                return
        else:
            self._wrong_mode_since = 0.0

        handler = {
            STATE_IDLE:           self._tick_idle,
            STATE_SEARCH:         self._tick_search,
            STATE_DETECTION_HOLD: self._tick_hold,
            STATE_APPROACH:       self._tick_approach,
            STATE_DROP:           self._tick_drop,
            STATE_DROP_HOLD:      self._tick_drop_hold,
            STATE_RTL:            self._tick_rtl,
            STATE_DONE:           self._tick_done,
        }.get(self._state)
        if handler:
            handler()

        self._publish_state()

    def _tick_idle(self):
        # Move into SEARCH when armed + in AUTO (FC executing uploaded mission)
        # OR in GUIDED (operator chose manual GUIDED orchestration).
        if self._enabled and self._armed and self._mode in ("AUTO", "GUIDED"):
            self._enter(STATE_SEARCH, reason=f"armed in {self._mode}")

    def _tick_search(self):
        if not (self._armed and self._mode in ("AUTO", "GUIDED")):
            self._enter(STATE_IDLE, reason="not armed / wrong mode for SEARCH")
            return
        t = self._last_target
        if t is None:
            return
        age = time.monotonic() - self._last_target_t
        if age > 1.5:
            return
        if t.confidence < self._min_confidence:
            return
        self._hold_start_t = time.monotonic()
        self._enter(STATE_DETECTION_HOLD,
                    reason=f"target conf={t.confidence:.2f} age={age:.2f}s")

    def _tick_hold(self):
        # During hold we expect continuous fresh detections. If they go stale,
        # treat as a false positive and resume SEARCH.
        age = time.monotonic() - self._last_target_t
        if age > 0.5:
            self._enter(STATE_SEARCH, reason=f"detection went stale ({age:.2f}s)")
            return
        if (time.monotonic() - self._hold_start_t) >= self._hold_time_s:
            # Compute drop point and request approach.
            t = self._last_target
            if t is None or self._drone_lat is None or self._drone_lon is None:
                self._enter(STATE_SEARCH, reason="missing drone/target position")
                return
            slat = t.position_geo.latitude
            slon = t.position_geo.longitude
            dlat, dlon = compute_drop_point(
                self._drone_lat, self._drone_lon, slat, slon, self._drop_offset_m,
            )
            self._drop_point = (dlat, dlon)
            self.get_logger().info(
                f"committed: survivor=({slat:.6f},{slon:.6f}) "
                f"drop_point=({dlat:.6f},{dlon:.6f})"
            )
            # Make sure we're in GUIDED so flyTo setpoints take effect
            # (AUTO will ignore them). If pilot is on TX-switch GUIDED
            # already this is a no-op.
            sm = String()
            sm.data = "GUIDED"
            self._pub_set_mode.publish(sm)
            self._enter(STATE_APPROACH, reason="hold confirmed")

    def _tick_approach(self):
        if self._drop_point is None or self._drone_lat is None:
            self._enter(STATE_IDLE, reason="approach lost setpoint or position")
            return
        # Publish setpoint intent every tick — downstream FC bridge will
        # de-dup. NaN altitude means "use current altitude" (FC behavior).
        sp = NavSatFix()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.latitude = self._drop_point[0]
        sp.longitude = self._drop_point[1]
        sp.altitude = float("nan")
        self._pub_flyto.publish(sp)
        # Arrival check.
        d = haversine_m(
            self._drone_lat, self._drone_lon,
            self._drop_point[0], self._drop_point[1],
        )
        if d <= self._approach_tol_m:
            self._enter(STATE_DROP, reason=f"arrived ({d:.1f}m)")

    def _tick_drop(self):
        if not self._payload_open:
            msg = String()
            msg.data = "open"
            self._pub_payload_cmd.publish(msg)
            self.get_logger().info("payload OPEN command issued")
            self._drop_open_t = time.monotonic()
        # Once we've seen the state change to open, move to hold-and-close.
        if self._payload_open:
            self._enter(STATE_DROP_HOLD, reason="payload acknowledged OPEN")

    def _tick_drop_hold(self):
        if (time.monotonic() - self._drop_open_t) >= self._drop_hold_s:
            msg = String()
            msg.data = "close"
            self._pub_payload_cmd.publish(msg)
            self.get_logger().info("payload CLOSE issued, requesting RTL")
            self._pub_rtl.publish(Empty())
            self._enter(STATE_RTL, reason="drop complete")

    def _tick_rtl(self):
        # Hard timeout so we don't hang in RTL forever if home is never set
        # or the drone can't reach home (bench test, GPS dropout, low batt
        # forcing a land elsewhere).
        elapsed = time.monotonic() - self._state_entered_t
        if elapsed >= self._rtl_max_s:
            self._enter(STATE_DONE,
                        reason=f"RTL timeout ({elapsed:.0f}s, no home arrival)")
            return
        if self._home_lat is None or self._drone_lat is None:
            return
        d = haversine_m(
            self._drone_lat, self._drone_lon, self._home_lat, self._home_lon,
        )
        if d <= self._home_tol_m:
            self._enter(STATE_DONE, reason=f"home reached ({d:.1f}m)")

    def _tick_done(self):
        pass

    # ── Helpers ──────────────────────────────────────────────

    def _enter(self, new_state, reason=""):
        if new_state == self._state:
            return
        old = self._state
        self._state = new_state
        self._state_entered_t = time.monotonic()
        self.get_logger().info(f"state: {old} → {new_state} ({reason})")

    def _publish_state(self):
        m = MissionState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self._state
        m.sub_state = self._mode
        # target_distance_m: distance to current target if known
        if self._drop_point is not None and self._drone_lat is not None:
            m.target_distance_m = float(haversine_m(
                self._drone_lat, self._drone_lon,
                self._drop_point[0], self._drop_point[1],
            ))
        else:
            m.target_distance_m = float("nan")
        m.altitude_agl_m = float("nan")
        m.battery_remaining = float("nan")
        m.vision_locked = self._last_target is not None and \
            (time.monotonic() - self._last_target_t) < 1.5
        # Healthy only if gimbal_controller reported True within the last
        # 5 s. Stale/absent (controller down, never started) → unhealthy,
        # so the dashboard SAR card shows a truthful gimbal dot instead of
        # the old hardcoded green.
        m.gimbal_healthy = bool(
            self._gimbal_healthy
            and (time.monotonic() - self._gimbal_health_t) < 5.0
        )
        m.gps_healthy = self._drone_lat is not None
        self._pub_state.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SarOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
