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

from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

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
        self._pub_att  = self.create_publisher(Vector3Stamped, "/vehicle/attitude", 10)
        self._pub_ned  = self.create_publisher(PointStamped,   "/vehicle/pose_ned", 10)
        self._pub_home = self.create_publisher(NavSatFix,      "/vehicle/home",      1)
        self._pub_gps  = self.create_publisher(NavSatFix,      "/vehicle/gps",      10)
        self._pub_arm  = self.create_publisher(Bool,           "/vehicle/armed",    10)

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
        self._mav = mavutil.mavlink_connection(
            port,
            baud=baud,
            autoreconnect=True,
            source_system=255,
            source_component=0,
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
                self._mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
                self._last_hb_t = now

            msg = self._mav.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue

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

    # ── GCS → FC forwarder (background thread) ────────────────────────
    #
    # Anything the GCS sends to UDP gcs_forward_listen is treated as raw
    # MAVLink bytes and written straight through to the FC serial. This is
    # how the SkyResQ dashboard commands the drone (ARM / MODE / GOTO) over
    # the 4G link.

    def _gcs_to_fc_loop(self):
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

    def _handle_heartbeat(self, msg):
        armed = bool(msg.base_mode & 128)   # MAV_MODE_FLAG_SAFETY_ARMED
        b = Bool()
        b.data = armed
        self._pub_arm.publish(b)


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
