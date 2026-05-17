#!/usr/bin/env python3
"""
MAVLink Bridge — ArduPilot to ROS 2.

Connects to the CubeBlack (ArduCopter) via pymavlink on TELEM2 and
republishes telemetry as generic ROS 2 topics so geo_localiser, gimbal,
and person_detector work without any px4_msgs dependency on the Pi.

Published topics
----------------
/vehicle/attitude   geometry_msgs/Vector3Stamped  roll/pitch/yaw (radians)
/vehicle/pose_ned   geometry_msgs/PointStamped     NED from home (m); z = -AGL
/vehicle/home       sensor_msgs/NavSatFix          home position (geodetic)
/vehicle/gps        sensor_msgs/NavSatFix          current GPS position
/vehicle/armed      std_msgs/Bool                  True when motors armed

Parameters
----------
connection_string  str    /dev/serial0            Serial UART or TCP/UDP URL.
                                                   Serial: /dev/serial0 (hardware)
                                                   TCP:    tcp:127.0.0.1:5760 (ArduPilot SITL)
                                                   UDP:    udp:127.0.0.1:14550
baud_rate          int    57600                    Baud rate — ignored for TCP/UDP connections.
target_sysid       int    1                        ArduCopter system ID
heartbeat_hz       float  1.0                      Rate to send GCS heartbeat to FC
stream_hz          int    10                       Rate to request from FC (REQUEST_DATA_STREAM)

# 4G/Tailscale MAVLink mirror — Pi rebroadcasts FC MAVLink to GCS via UDP.
# Lets the SkyResQ dashboard use the 4G link as the primary telemetry path
# (with SiK as automatic fallback) so the operator gets 10 Hz updates with
# minimal packet loss instead of 4 Hz over SiK 433 MHz.
gcs_forward_ip      str    ""                      Target GCS IP — leave empty to disable forwarding
gcs_forward_port    int    14550                   Standard MAVLink UDP port on the GCS
gcs_forward_listen  int    14551                   Local UDP bind port for GCS→FC commands
"""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit(
        "pymavlink not found — install with: pip3 install pymavlink"
    )

EARTH_R = 6_371_000.0  # metres


def _flat_earth_ned(home_lat_deg, home_lon_deg, lat_deg, lon_deg):
    """Return (north_m, east_m) offset of (lat, lon) from home."""
    dlat = math.radians(lat_deg - home_lat_deg)
    dlon = math.radians(lon_deg - home_lon_deg)
    north = dlat * EARTH_R
    east  = dlon * EARTH_R * math.cos(math.radians(home_lat_deg))
    return north, east


class MavlinkBridgeNode(Node):

    def __init__(self):
        super().__init__("mavlink_bridge")

        self.declare_parameter("connection_string",   "/dev/serial0")
        self.declare_parameter("baud_rate",           57600)
        self.declare_parameter("target_sysid",        1)
        self.declare_parameter("heartbeat_hz",        1.0)
        self.declare_parameter("stream_hz",           10)
        self.declare_parameter("gcs_forward_ip",      "")
        self.declare_parameter("gcs_forward_port",    14550)
        self.declare_parameter("gcs_forward_listen",  14551)

        port      = self.get_parameter("connection_string").value
        baud      = int(self.get_parameter("baud_rate").value)
        self._sysid  = int(self.get_parameter("target_sysid").value)
        hb_hz     = float(self.get_parameter("heartbeat_hz").value)
        stream_hz = int(self.get_parameter("stream_hz").value)
        self._fwd_ip   = self.get_parameter("gcs_forward_ip").value
        self._fwd_port = int(self.get_parameter("gcs_forward_port").value)
        fwd_listen     = int(self.get_parameter("gcs_forward_listen").value)

        # ── Publishers ─────────────────────────────────────────────────
        # /vehicle/home uses TRANSIENT_LOCAL durability so any subscriber
        # that connects after the FC has already sent its one-shot
        # HOME_POSITION still receives the latched value (otherwise the
        # geo_localiser silently misses it and can't compute lat/lon).
        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_att  = self.create_publisher(Vector3Stamped, "/vehicle/attitude", 10)
        self._pub_ned  = self.create_publisher(PointStamped,   "/vehicle/pose_ned", 10)
        self._pub_home = self.create_publisher(NavSatFix,      "/vehicle/home",     home_qos)
        self._pub_gps  = self.create_publisher(NavSatFix,      "/vehicle/gps",      10)
        self._pub_arm  = self.create_publisher(Bool,           "/vehicle/armed",    10)
        self._pub_mode = self.create_publisher(String,         "/vehicle/mode",     10)
        self._last_mode_published: str | None = None

        # ── Payload bridge ────────────────────────────────────────────
        # Dashboard sends MAV_CMD_USER_1 over SiK (primary) or 4G/Tailscale
        # mirror (secondary). ArduPilot routes the message to TELEM2 because
        # the target_component != FC's compid. We translate to a /payload/cmd
        # string ("open"|"close"|"toggle") which payload_servo consumes.
        #
        # Convention: param1 selects the action.
        #   1.0 -> "open"   (release / 180°)
        #   0.0 -> "close"  (grab / 0°)
        #   2.0 -> "toggle"
        self._pub_payload_cmd = self.create_publisher(String, "/payload/cmd", 10)
        payload_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._payload_state: bool | None = None
        self.create_subscription(
            Bool, "/payload/state", self._on_payload_state, payload_state_qos
        )

        # ── Mission orchestrator → FC command bridge ─────────────────
        # sar_orchestrator publishes intent on /mission/fly_to + /mission/cmd_rtl
        # + /mission/set_mode; this node translates each into the matching
        # MAVLink message and writes it to the FC serial.
        from sensor_msgs.msg import NavSatFix as _NavSatFix
        from std_msgs.msg import Empty as _Empty
        self.create_subscription(_NavSatFix, "/mission/fly_to", self._on_mission_fly_to, 10)
        self.create_subscription(_Empty, "/mission/cmd_rtl", self._on_mission_rtl, 10)
        self.create_subscription(String, "/mission/set_mode", self._on_mission_set_mode, 10)

        # /mission/enable published when the dashboard sends MAV_CMD_USER_2.
        # Latched so a late-spawning sar_orchestrator gets the last value.
        mission_enable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_mission_enable = self.create_publisher(
            Bool, "/mission/enable", mission_enable_qos
        )
        # /gimbal/cmd/set_attitude published when the dashboard sends
        # MAV_CMD_USER_3 (param1=pitch deg, param2=yaw deg) for manual
        # operator gimbal control. Purely additive — the autonomous
        # look_at_pixel tracking path is untouched.
        self._pub_gimbal_setpoint = self.create_publisher(
            Vector3Stamped, "/gimbal/cmd/set_attitude", 10
        )
        # Rate-limit fly_to writes — the orchestrator publishes every tick
        # (~5 Hz) and the FC only needs the current setpoint; we throttle so
        # we don't saturate the 57600-baud serial.
        self._last_flyto_t = 0.0
        self._flyto_min_interval = 0.5  # 2 Hz max
        # Track current AGL so fly_to can substitute it when the orchestrator
        # passes NaN ("hold current altitude"). Sending alt=0+ignore-bit was
        # version-fragile — some ArduCopter builds did honor the ignore-z bit,
        # some commanded the copter toward alt=0 (ground). An explicit valid
        # altitude with z-bit USED is what every reliable GUIDED path does.
        self._current_relative_alt: float | None = None

        # ── Home state ─────────────────────────────────────────────────
        self._home_lat:   float | None = None
        self._home_lon:   float | None = None
        self._home_alt:   float        = 0.0
        self._home_pub_t: float        = 0.0

        # ── MAVLink connection ─────────────────────────────────────────
        is_serial = not (port.startswith("tcp:") or port.startswith("udp:"))
        self.get_logger().info(
            f"Connecting to ArduPilot — {port}"
            + (f" @ {baud} baud" if is_serial else " (TCP/UDP)")
        )
        # Identify as a separate "system 2" companion-computer
        # (MAV_COMP_ID_ONBOARD_COMPUTER=191). Using a distinct sysid (not 1,
        # which would collide with the FC) lets ArduPilot's MAVLink router
        # cleanly forward GCS→(2,191) packets to TELEM2 — when we used
        # sysid=1 same as the FC, the routing collapsed the two entries and
        # the forward never happened.
        self._mav = mavutil.mavlink_connection(
            port,
            baud=baud,
            autoreconnect=True,
            source_system=2,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )

        hb = self._mav.wait_heartbeat(timeout=30)
        if hb is None:
            self.get_logger().error(
                "No heartbeat from FC within 30 s — check TELEM2 wiring and SERIAL2_PROTOCOL=2"
            )
        else:
            self.get_logger().info(
                f"FC heartbeat — sysid={self._mav.target_system} "
                f"compid={self._mav.target_component} "
                f"autopilot={hb.autopilot}"
            )
            # Request all telemetry streams at stream_hz
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                stream_hz,
                1,  # start
            )
            # Request HOME_POSITION once
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                0, 0, 0, 0, 0, 0,
            )

        self._hb_interval = 1.0 / max(hb_hz, 0.5)
        self._last_hb_t   = 0.0

        # Diagnostic: aggregate msg_type counts, log every 5 s.
        # Lets us answer "does FC forward COMMAND_LONG (and other GCS→Pi
        # routed messages) over TELEM2 at all?" without grepping every frame.
        self._msg_type_counts: dict[str, int] = {}
        self._last_stats_t = 0.0

        # ── GCS UDP forward (optional) ─────────────────────────────────
        self._fwd_sock: socket.socket | None = None
        self._fwd_thread: threading.Thread | None = None
        if self._fwd_ip:
            self._fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self._fwd_sock.bind(("0.0.0.0", fwd_listen))
                self._fwd_sock.settimeout(0.2)
                self.get_logger().info(
                    f"MAVLink mirror → {self._fwd_ip}:{self._fwd_port} "
                    f"(listening on UDP {fwd_listen} for GCS→FC)"
                )
            except OSError as exc:
                self.get_logger().warning(
                    f"GCS forward disabled — could not bind UDP {fwd_listen}: {exc}"
                )
                self._fwd_sock = None

        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._mavlink_loop, daemon=True)
        self._thread.start()

        if self._fwd_sock is not None:
            self._fwd_thread = threading.Thread(target=self._gcs_to_fc_loop, daemon=True)
            self._fwd_thread.start()

        self.get_logger().info("MAVLink bridge running")

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

    # ── MAVLink read loop (background thread — pymavlink is synchronous) ──

    def _mavlink_loop(self):
        while not self._stop.is_set():
            now = time.monotonic()
            if now - self._last_hb_t >= self._hb_interval:
                # Companion-computer heartbeat. Lets ArduPilot learn that
                # compid 191 lives on TELEM2 and route inbound packets here.
                self._mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                self._last_hb_t = now

            msg = self._mav.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

            # Loud log on routing-relevant inbound events so we can tell
            # at a glance whether FC forwards GCS→Pi commands over SiK.
            if msg_type in ("COMMAND_LONG", "COMMAND_INT", "COMMAND_ACK",
                            "HEARTBEAT", "STATUSTEXT"):
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()
                # Skip our own emitted heartbeats (sysid=2 compid=191)
                if not (msg_type == "HEARTBEAT" and src_sys == 2 and src_comp == 191):
                    extras = ""
                    if msg_type == "COMMAND_LONG":
                        extras = (f" cmd={int(msg.command)} "
                                  f"tgt_sys={msg.target_system} tgt_comp={msg.target_component} "
                                  f"p1={msg.param1}")
                    elif msg_type == "COMMAND_ACK":
                        extras = f" cmd={int(msg.command)} result={int(msg.result)}"
                    self.get_logger().info(
                        f"rx {msg_type} from src=({src_sys},{src_comp}){extras}"
                    )

            # Mirror the raw frame to the GCS over UDP (Tailscale)
            if self._fwd_sock is not None:
                try:
                    raw = msg.get_msgbuf()
                    if raw:
                        self._fwd_sock.sendto(bytes(raw), (self._fwd_ip, self._fwd_port))
                except OSError:
                    pass   # transient — next packet will retry

            if msg_type == "ATTITUDE":
                self._handle_attitude(msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self._handle_global_pos(msg)
            elif msg_type == "HOME_POSITION":
                self._handle_home_position(msg)
            elif msg_type == "HEARTBEAT":
                if msg.get_srcSystem() == self._sysid:
                    self._handle_heartbeat(msg)
            elif msg_type == "COMMAND_LONG":
                self._handle_command_long(msg)

    # ── GCS → FC forwarder (background thread) ────────────────────────
    #
    # Anything the GCS sends to UDP gcs_forward_listen is treated as raw
    # MAVLink bytes and written straight through to the FC serial. This is
    # how the SkyResQ dashboard commands the drone (ARM / MODE / GOTO) over
    # the 4G link.

    def _gcs_to_fc_loop(self):
        # Separate MAVLink parser instance for UDP-injected bytes so partial
        # frames from the UDP socket don't corrupt the serial parser buffer.
        udp_parser = mavutil.mavlink.MAVLink(None)
        udp_parser.robust_parsing = True

        while not self._stop.is_set():
            try:
                data, addr = self._fwd_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().debug(f"UDP recv error: {exc}")
                time.sleep(0.1)
                continue

            if not data:
                continue

            # Parse first so we can intercept payload commands locally without
            # round-tripping through the FC (ArduPilot's MAVLink router does
            # not reliably forward COMMAND_LONG with target_component=191).
            intercepted = False
            try:
                msgs = udp_parser.parse_buffer(data)
            except Exception:
                msgs = None
            if msgs:
                for m in msgs:
                    if m.get_type() != "COMMAND_LONG":
                        continue
                    cmd = int(getattr(m, "command", -1))
                    if cmd == mavutil.mavlink.MAV_CMD_USER_1:
                        self.get_logger().info("payload MAV_CMD_USER_1 via UDP — handled locally")
                        self._handle_command_long(m)
                        intercepted = True
                    elif cmd == mavutil.mavlink.MAV_CMD_USER_2:
                        self.get_logger().info("mission MAV_CMD_USER_2 via UDP — handled locally")
                        self._handle_command_long(m)
                        intercepted = True
                    elif cmd == mavutil.mavlink.MAV_CMD_USER_3:
                        self.get_logger().info("gimbal MAV_CMD_USER_3 via UDP — handled locally")
                        self._handle_command_long(m)
                        intercepted = True

            if intercepted:
                # Don't relay the payload command to FC — it's a Pi-only command.
                continue

            try:
                # Write raw bytes to the underlying pymavlink connection.
                # write() handles both serial and TCP/UDP transports.
                self._mav.write(data)
                # Latch the GCS endpoint we last heard from. Useful if the
                # parameter was empty and someone is probing us.
                if not self._fwd_ip:
                    self._fwd_ip = addr[0]
            except Exception as exc:
                self.get_logger().warning(f"FC write failed: {exc}")

    # ── Message handlers ───────────────────────────────────────────────

    def _handle_attitude(self, msg):
        m = Vector3Stamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.vector.x = float(msg.roll)
        m.vector.y = float(msg.pitch)
        m.vector.z = float(msg.yaw)
        self._pub_att.publish(m)

    def _handle_global_pos(self, msg):
        lat     = msg.lat / 1e7
        lon     = msg.lon / 1e7
        alt_msl = msg.alt / 1000.0
        agl     = msg.relative_alt / 1000.0   # metres AGL
        self._current_relative_alt = agl

        stamp = self.get_clock().now().to_msg()

        # GPS topic — always published
        gps = NavSatFix()
        gps.header.stamp    = stamp
        gps.header.frame_id = "map"
        gps.latitude  = lat
        gps.longitude = lon
        gps.altitude  = alt_msl
        gps.status.status = 0
        self._pub_gps.publish(gps)

        # NED topic — only once home is known
        if self._home_lat is not None:
            north, east = _flat_earth_ned(self._home_lat, self._home_lon, lat, lon)
            ned = PointStamped()
            ned.header.stamp    = stamp
            ned.header.frame_id = "map"
            ned.point.x = north
            ned.point.y = east
            ned.point.z = -agl   # NED: z negative when above ground
            self._pub_ned.publish(ned)
        else:
            # Bootstrap home from the first position report if HOME_POSITION
            # hasn't arrived yet (e.g. bench test with no outdoor GPS).
            self._set_home(lat, lon, alt_msl)

    def _handle_home_position(self, msg):
        lat = msg.latitude  / 1e7
        lon = msg.longitude / 1e7
        alt = msg.altitude  / 1000.0   # mm → m MSL
        self._set_home(lat, lon, alt)

    def _set_home(self, lat, lon, alt):
        if self._home_lat is None:
            self._home_lat = lat
            self._home_lon = lon
            self._home_alt = alt
            self.get_logger().info(
                f"Home set: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f} m MSL"
            )

        # Re-publish home every 5 s so late subscribers pick it up
        now = time.monotonic()
        if now - self._home_pub_t >= 5.0:
            home = NavSatFix()
            home.header.stamp    = self.get_clock().now().to_msg()
            home.header.frame_id = "map"
            home.latitude  = self._home_lat
            home.longitude = self._home_lon
            home.altitude  = self._home_alt
            home.status.status = 0
            self._pub_home.publish(home)
            self._home_pub_t = now

    # ArduCopter mode IDs → names (subset used in the SAR mission).
    _COPTER_MODE = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
        5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD",
        17: "BRAKE", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
    }

    def _handle_heartbeat(self, msg):
        armed = bool(msg.base_mode & 128)   # MAV_MODE_FLAG_SAFETY_ARMED
        b = Bool()
        b.data = armed
        self._pub_arm.publish(b)
        # Publish flight mode (used by sar_orchestrator + dashboard).
        mode_name = self._COPTER_MODE.get(int(msg.custom_mode), f"MODE_{int(msg.custom_mode)}")
        if mode_name != self._last_mode_published:
            self._last_mode_published = mode_name
            sm = String()
            sm.data = mode_name
            self._pub_mode.publish(sm)

    # ── Payload command routing ────────────────────────────────────────

    def _handle_command_long(self, msg):
        # MAV_CMD_USER_1 = 31010 — repurposed for payload toggle.
        try:
            cmd_id = int(msg.command)
        except Exception:
            return
        # Log every inbound COMMAND_LONG so we can see what's being routed.
        self.get_logger().info(
            f"COMMAND_LONG seen: cmd={cmd_id} target_sys={msg.target_system} "
            f"target_comp={msg.target_component} src_sys={msg.get_srcSystem()} "
            f"src_comp={msg.get_srcComponent()} param1={msg.param1}"
        )
        # MAV_CMD_USER_2 = 31011 — repurposed for SAR autonomy enable/disable.
        # param1 1.0 = enable, 0.0 = disable.
        if cmd_id == mavutil.mavlink.MAV_CMD_USER_2:
            enable = float(getattr(msg, "param1", 0.0)) > 0.5
            self.get_logger().info(f"mission enable from GCS: {enable}")
            b = Bool()
            b.data = enable
            self._pub_mission_enable.publish(b)
            self._send_ack(cmd_id, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return
        # MAV_CMD_USER_3 = 31012 — repurposed for manual gimbal control.
        # param1 = pitch deg, param2 = yaw deg. Additive: just publishes a
        # setpoint the gimbal_controller already knows how to slew to; the
        # autonomous look_at_pixel path is unaffected.
        if cmd_id == mavutil.mavlink.MAV_CMD_USER_3:
            pitch = float(getattr(msg, "param1", 0.0))
            yaw = float(getattr(msg, "param2", 0.0))
            self.get_logger().info(
                f"gimbal setpoint from GCS: pitch={pitch:.1f} yaw={yaw:.1f}"
            )
            v = Vector3Stamped()
            v.header.stamp = self.get_clock().now().to_msg()
            v.vector.x = 0.0
            v.vector.y = pitch
            v.vector.z = yaw
            self._pub_gimbal_setpoint.publish(v)
            self._send_ack(cmd_id, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return
        if cmd_id != mavutil.mavlink.MAV_CMD_USER_1:
            return
        # Translate param1 -> action
        action_map = {1.0: "open", 0.0: "close", 2.0: "toggle"}
        param1 = float(getattr(msg, "param1", 0.0))
        # Tolerant match
        action = None
        for key, name in action_map.items():
            if abs(param1 - key) < 0.25:
                action = name
                break
        if action is None:
            self.get_logger().warning(
                f"payload MAV_CMD_USER_1 with unknown param1={param1}"
            )
            self._send_ack(cmd_id, mavutil.mavlink.MAV_RESULT_DENIED)
            return
        self.get_logger().info(
            f"payload command from GCS: {action} (param1={param1})"
        )
        out = String()
        out.data = action
        self._pub_payload_cmd.publish(out)
        self._send_ack(cmd_id, mavutil.mavlink.MAV_RESULT_ACCEPTED)

    def _send_ack(self, cmd_id, result):
        try:
            self._mav.mav.command_ack_send(cmd_id, result)
        except Exception as exc:
            self.get_logger().debug(f"COMMAND_ACK send failed: {exc}")

    # ── Orchestrator → FC ─────────────────────────────────────────────

    # Mode names → ArduCopter custom_mode ids (mirror of _COPTER_MODE)
    _MODE_ID_FROM_NAME = {
        "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
        "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "POSHOLD": 16,
        "BRAKE": 17, "GUIDED_NOGPS": 20, "SMART_RTL": 21,
    }

    def _on_mission_fly_to(self, msg):
        # Rate limit — see comment in constructor.
        now = time.monotonic()
        if now - self._last_flyto_t < self._flyto_min_interval:
            return
        self._last_flyto_t = now
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(msg.altitude)
        if not (lat or lon):
            return
        # ArduPilot SET_POSITION_TARGET_GLOBAL_INT type_mask. We use the
        # full position triple (x,y,z bits CLEAR) — every reliable GUIDED
        # implementation does this. Tell FC to ignore vel (bits 3-5), accel
        # (6-8), yaw (bit 10), yaw_rate (bit 11). DO NOT set bit 9 (FORCE).
        # bits set: 3,4,5,6,7,8,10,11 = 0xDF8
        # ref: https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
        type_mask = 0xDF8
        if math.isnan(alt):
            # Orchestrator says "hold current altitude". Substitute the real
            # current AGL with the z-bit USED (not the ignore-z trick). Some
            # ArduCopter builds did not honor the ignore-altitude bit and
            # commanded the copter toward alt=0 (ground) instead — confirmed
            # in SITL when the approach drone never moved horizontally.
            if self._current_relative_alt is None:
                self.get_logger().warning(
                    "fly_to: no current altitude yet, dropping setpoint"
                )
                return
            alt = float(self._current_relative_alt)
        # pymavlink's wait_heartbeat sets target_system but in 2.4.x sometimes
        # leaves target_component=0 even when the autopilot's heartbeat is
        # srcComp=1. ArduCopter doesn't filter SET_POSITION_TARGET by
        # component, but DO_REPOSITION and others do — force the autopilot
        # component (1) for every FC-bound command to be safe.
        tgt_sys = self._mav.target_system
        tgt_comp = self._mav.target_component or 1
        try:
            self._mav.mav.set_position_target_global_int_send(
                0,                                  # time_boot_ms
                tgt_sys, tgt_comp,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt),
                0.0, 0.0, 0.0,    # vel x/y/z (ignored)
                0.0, 0.0, 0.0,    # acc x/y/z (ignored)
                0.0, 0.0,         # yaw, yaw_rate (ignored)
            )
            # Log first send + every 5 s so we can see it's actually firing.
            if (now - getattr(self, "_last_flyto_log", 0.0)) > 5.0:
                self.get_logger().info(
                    f"fly_to → ({lat:.6f},{lon:.6f}) alt={alt:.1f}m "
                    f"sys={tgt_sys} comp={tgt_comp}"
                )
                self._last_flyto_log = now
        except Exception as exc:
            self.get_logger().warning(f"fly_to send failed: {exc}")

    def _on_mission_rtl(self, _msg):
        try:
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0,
                0, 0, 0, 0, 0, 0, 0,
            )
            self.get_logger().info("RTL command sent")
        except Exception as exc:
            self.get_logger().warning(f"RTL send failed: {exc}")

    def _on_mission_set_mode(self, msg):
        name = (msg.data or "").strip().upper()
        mode_id = self._MODE_ID_FROM_NAME.get(name)
        if mode_id is None:
            self.get_logger().warning(f"unknown flight mode requested: {name!r}")
            return
        try:
            self._mav.mav.set_mode_send(
                self._mav.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.get_logger().info(f"set_mode → {name} (id={mode_id})")
        except Exception as exc:
            self.get_logger().warning(f"set_mode failed: {exc}")

    def _on_payload_state(self, msg: Bool):
        # Emit NAMED_VALUE_INT("PLDOPEN", 0|1) upstream so dashboard can
        # render the current state regardless of which link is up.
        is_open = 1 if msg.data else 0
        if self._payload_state == is_open:
            return
        self._payload_state = is_open
        try:
            self._mav.mav.named_value_int_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
                b"PLDOPEN",
                is_open,
            )
        except Exception as exc:
            self.get_logger().debug(f"NAMED_VALUE_INT PLDOPEN failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
