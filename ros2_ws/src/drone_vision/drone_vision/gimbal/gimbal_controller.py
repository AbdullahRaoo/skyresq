#!/usr/bin/env python3
"""
Gimbal Controller — XF Robot Z-1 Mini.

Drives the physical Z-1 Mini gimbal over its TCP control port using the
XFRobot binary protocol. Subscribes to /gimbal/cmd/look_at_pixel (the same
50 Hz pixel-error topic produced by visual_servo) and converts each error
sample into an absolute Euler-angle setpoint that is sent to the gimbal at
command_rate_hz. Parsed status packets from the gimbal are republished as
/gimbal/state (Vector3Stamped, degrees) matching the contract gimbal_sim
already satisfies.

Parameters
----------
gimbal_host        str    192.168.144.108
gimbal_port        int    2332
command_rate_hz    float  50.0       outbound Euler-angle command rate
state_rate_hz      float  20.0       inbound state polling rate (recv loop)
backend            str    tcp        tcp (default) | sim (proxy to gimbal_sim)
pixel_gain_yaw     float  30.0       deg of slew per unit normalised pixel error
pixel_gain_pitch   float  20.0       deg of slew per unit normalised pixel error
yaw_min_deg        float  -180.0
yaw_max_deg        float  +180.0
pitch_min_deg      float  -90.0
pitch_max_deg      float  +30.0
reconnect_s        float  2.0

XFRobot framing (verified against ArduPilot AP_Mount_XFRobot.cpp)
-----------------------------------------------------------------
  Out header: 0xA8 0xE5      In header: 0x8A 0x5E      Version: 0x02

  Every command (including ANGLE_CONTROL) shares one 72-byte frame:
    bytes 0-1   header (0xA8, 0xE5)
    bytes 2-3   length uint16 LE = 72
    byte  4     version = 0x02
    bytes 5-6   roll_control  int16 LE centi-deg (-18000..+18000)
    bytes 7-8   pitch_control int16 LE centi-deg (-9000..+9000)
    bytes 9-10  yaw_control   int16 LE centi-deg (-18000..+18000)
    byte 11     status — Bit0:INS valid, Bit2:control values valid
    bytes 12-23 vehicle attitude (3x int16 centi-deg) + accel (3x int16 cm/s²)
    bytes 24-29 vehicle velocity NED (3x int16 decimeter/s)
    byte 30     request_code = 0x01 (asks gimbal to return sub-frame)
    bytes 31-36 reserved (zeros)
    byte 37     sub_header = 0x01
    bytes 38-49 vehicle lat/lon/alt (3x int32 — lat/lon 1e7, alt mm AMSL)
    byte 50     gps_num_sats uint8
    bytes 51-54 gps_week_ms uint32 LE
    bytes 55-56 gps_week    uint16 LE
    bytes 57-60 alt_rel int32 LE (mm above home)
    bytes 61-68 reserved2 (zeros)
    byte 69     order code (0x10 = ANGLE_CONTROL)
    bytes 70-71 CRC-16/XMODEM (poly 0x1021, init 0, no reflection) HIGH byte first

  Inbound reply packets have the same 4-byte header+length prefix but a
  different main+sub body shape — we only need three fields from it:
    bytes 18-19 roll_abs_cd  int16 LE centi-deg
    bytes 20-21 pitch_abs_cd int16 LE centi-deg
    bytes 22-23 yaw_abs_cd   uint16 LE centi-deg (0..36000)
"""

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Vector3Stamped


# ── Protocol constants ────────────────────────────────────────────────
HDR_OUT = bytes([0xA8, 0xE5])
HDR_IN  = bytes([0x8A, 0x5E])
PROTO_VERSION = 0x02

# Function-order codes (subset — see AP_Mount_XFRobot.h for full list)
ORDER_ANGLE_CONTROL  = 0x10
ORDER_HEAD_FOLLOW    = 0x12
ORDER_TRACK          = 0x17
ORDER_CLICK_TO_AIM   = 0x1A
ORDER_SHUTTER        = 0x20

# Status byte bits inside the send packet (byte 11)
STATUS_INS_VALID     = 1 << 0
STATUS_CTRL_VALID    = 1 << 2

# Send packet has fixed 72-byte size (70 main+sub frame + 2 CRC)
SEND_PACKET_SIZE = 72
# struct.pack format for the 70-byte main+sub frame (no CRC); little-endian.
# Counts: 2+2+1+6+1+6+6+6+1+6+1+4+4+4+1+4+2+4+8+1 = 70
_MAIN_FMT = "<BBHBhhhBhhhhhhhhhB6xBiiiBIHi8xB"

# CRC-16/XMODEM (poly 0x1021, init 0, no reflection, no xor-out)
_CRC_POLY = 0x1021
_CRC_INIT = 0x0000


def _crc16_xmodem(data: bytes) -> int:
    crc = _CRC_INIT
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC_POLY) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_angle_control_packet(
    pitch_deg: float,
    yaw_deg:   float,
    roll_deg:  float = 0.0,
) -> bytes:
    """Build a 72-byte ANGLE_CONTROL (0x10) packet for the Z-1 Mini.

    No vehicle telemetry (INS bit not set) — gimbal will use its own IMU.
    For full-FC integration the AHRS / GPS fields can be filled in.
    """
    roll_cd  = max(-18000, min(18000, int(round(roll_deg  * 100))))
    pitch_cd = max( -9000, min( 9000, int(round(pitch_deg * 100))))
    yaw_cd   = max(-18000, min(18000, int(round(yaw_deg   * 100))))

    main = struct.pack(
        _MAIN_FMT,
        HDR_OUT[0], HDR_OUT[1],     # header1, header2
        SEND_PACKET_SIZE,            # length (total packet bytes incl CRC)
        PROTOCOL_VERSION_BYTE,       # version
        roll_cd, pitch_cd, yaw_cd,   # control values (centi-deg)
        STATUS_CTRL_VALID,           # status — only "control valid"
        0, 0, 0,                     # vehicle abs roll/pitch/yaw (no INS)
        0, 0, 0,                     # accel N/E/U
        0, 0, 0,                     # vel N/E/U
        0x01,                        # request_code — ask for sub-frame reply
        0x01,                        # sub_header
        0, 0, 0,                     # lon, lat, alt_amsl (no GPS)
        0,                           # gps_num_sats
        0, 0,                        # gps_week_ms, gps_week
        0,                           # alt_rel
        ORDER_ANGLE_CONTROL,         # byte 69: order
    )
    assert len(main) == 70, f"main frame size mismatch: {len(main)}"
    crc = _crc16_xmodem(main)
    # CRC is HIGH byte first then LOW byte (NOT little-endian as a uint16)
    return main + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


# Convenience constant referenced inside build_angle_control_packet
PROTOCOL_VERSION_BYTE = PROTO_VERSION


def parse_state_packet(buf: bytearray):
    """
    Try to extract one inbound packet from a streaming buffer.

    The Z-1 Mini's reply frame is also length-prefixed at bytes 2-3
    (uint16 LE = total packet size). We pull the cached camera attitude
    out of bytes 18-23 (roll_abs_cd, pitch_abs_cd, yaw_abs_cd as int16/uint16
    centi-degrees).

    Returns (consumed_bytes, parsed_dict_or_None) per parser convention.
    """
    idx = buf.find(HDR_IN)
    if idx < 0:
        # No header — discard all but the last byte (could be partial header)
        return max(0, len(buf) - 1), None
    if idx > 0:
        return idx, None

    # Header found at offset 0. Need at least 4 bytes to read length.
    if len(buf) < 4:
        return 0, None

    total = struct.unpack_from("<H", buf, 2)[0]    # length = whole packet
    if total < 24 or total > 200:
        # Sanity check — drop the header byte and resync
        return 1, None
    if len(buf) < total:
        return 0, None

    body = bytes(buf[: total - 2])
    crc_pkt = (buf[total - 2] << 8) | buf[total - 1]   # HIGH byte first
    if _crc16_xmodem(body) != crc_pkt:
        return 1, None    # Bad CRC, drop header and re-sync

    # Camera attitude — all int16 centi-degrees at bytes 18-19, 20-21, 22-23.
    # ArduPilot's .h declares yaw_abs_cd as uint16 with range 0..36000, but
    # the live firmware in the Z-1 Mini we have here actually sends signed
    # int16 — confirmed on bench: a -30° yaw came back as 62500 raw (uint16
    # interpretation) = -3036 (int16 interpretation = correct -30.36°).
    roll_cd, pitch_cd, yaw_cd = struct.unpack_from("<hhh", buf, 18)
    return total, {
        "roll_deg":  roll_cd  / 100.0,
        "pitch_deg": pitch_cd / 100.0,
        "yaw_deg":   yaw_cd   / 100.0,
        "raw_order": buf[69] if total > 70 else None,
    }


# ── Node ──────────────────────────────────────────────────────────────

class GimbalControllerNode(Node):

    def __init__(self):
        super().__init__("gimbal_controller")

        self.declare_parameter("gimbal_host",       "192.168.144.108")
        self.declare_parameter("gimbal_port",       2332)
        self.declare_parameter("command_rate_hz",   50.0)
        self.declare_parameter("state_rate_hz",     20.0)
        self.declare_parameter("backend",           "tcp")
        self.declare_parameter("pixel_gain_yaw",    30.0)
        self.declare_parameter("pixel_gain_pitch",  20.0)
        self.declare_parameter("yaw_min_deg",       -180.0)
        self.declare_parameter("yaw_max_deg",        180.0)
        self.declare_parameter("pitch_min_deg",     -90.0)
        self.declare_parameter("pitch_max_deg",      30.0)
        self.declare_parameter("reconnect_s",        2.0)
        # Initial pose — gimbal slews here on startup before any pixel cmds
        # arrive. Default nadir for flight ops; bench tests typically set
        # initial_pitch_deg ~= -10 so the camera looks forward at people.
        self.declare_parameter("initial_pitch_deg", -90.0)
        self.declare_parameter("initial_yaw_deg",     0.0)

        self._host       = self.get_parameter("gimbal_host").value
        self._port       = int(self.get_parameter("gimbal_port").value)
        cmd_hz           = float(self.get_parameter("command_rate_hz").value)
        self._backend    = self.get_parameter("backend").value
        self._k_yaw      = float(self.get_parameter("pixel_gain_yaw").value)
        self._k_pitch    = float(self.get_parameter("pixel_gain_pitch").value)
        self._yaw_min    = float(self.get_parameter("yaw_min_deg").value)
        self._yaw_max    = float(self.get_parameter("yaw_max_deg").value)
        self._pitch_min  = float(self.get_parameter("pitch_min_deg").value)
        self._pitch_max  = float(self.get_parameter("pitch_max_deg").value)
        self._reconn_s   = float(self.get_parameter("reconnect_s").value)

        # ── Setpoint state (commanded gimbal pose, NED-like body frame) ──
        # pitch = down +, yaw = right + (matches gimbal_sim conventions)
        init_pitch = float(self.get_parameter("initial_pitch_deg").value)
        init_yaw   = float(self.get_parameter("initial_yaw_deg").value)
        self._target_pitch = max(self._pitch_min, min(self._pitch_max, init_pitch))
        self._target_yaw   = max(self._yaw_min,   min(self._yaw_max,   init_yaw))
        # Seed state with the same values so /gimbal/state reads sensibly
        # before the gimbal sends its first reply
        self._state_pitch  = self._target_pitch
        self._state_yaw    = self._target_yaw
        self._state_roll   = 0.0
        self._lock = threading.Lock()

        # ── ROS interfaces ─────────────────────────────────────────────
        self._pub_state = self.create_publisher(Vector3Stamped, "/gimbal/state", 10)
        self.create_subscription(
            PointStamped, "/gimbal/cmd/look_at_pixel", self._on_pixel_cmd, 50
        )

        # Outbound command timer (rate-limits writes regardless of inbound rate)
        self.create_timer(1.0 / max(cmd_hz, 1.0), self._send_command_tick)

        # ── TCP I/O thread ─────────────────────────────────────────────
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        if self._backend == "tcp":
            self._io_thread = threading.Thread(target=self._io_loop, daemon=True)
            self._io_thread.start()
            self.get_logger().info(
                f"Gimbal controller (tcp) → {self._host}:{self._port} "
                f"| cmd={cmd_hz:.0f} Hz"
            )
        else:
            self._io_thread = None
            self.get_logger().info(
                "Gimbal controller (sim backend) — no TCP, /gimbal/state echoes targets"
            )

    def destroy_node(self):
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        super().destroy_node()

    # ── ROS callbacks ─────────────────────────────────────────────────

    def _on_pixel_cmd(self, msg: PointStamped):
        """Update target Euler angles from normalised pixel error."""
        ex = float(msg.point.x)   # -1..1, +ve = target right of centre
        ey = float(msg.point.y)   # -1..1, +ve = target below centre

        with self._lock:
            yaw   = self._target_yaw   + ex * self._k_yaw
            pitch = self._target_pitch - ey * self._k_pitch
            self._target_yaw   = max(self._yaw_min,   min(self._yaw_max,   yaw))
            self._target_pitch = max(self._pitch_min, min(self._pitch_max, pitch))

    def _send_command_tick(self):
        """Emit one EULER_ANGLE_CONTROL packet (tcp) or echo state (sim)."""
        with self._lock:
            pitch = self._target_pitch
            yaw   = self._target_yaw

        if self._backend == "sim":
            # No physical hardware — echo target as current state for
            # downstream consumers (geo_localiser, debug).
            with self._lock:
                self._state_pitch = pitch
                self._state_yaw   = yaw
            self._publish_state()
            return

        sock = self._sock
        if sock is None:
            return
        try:
            sock.sendall(build_angle_control_packet(pitch, yaw))
        except OSError as exc:
            self.get_logger().warning(f"Gimbal send failed: {exc}")
            try:
                sock.close()
            except Exception:
                pass
            self._sock = None

    def _publish_state(self):
        m = Vector3Stamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "gimbal"
        with self._lock:
            m.vector.x = self._state_roll
            m.vector.y = self._state_pitch
            m.vector.z = self._state_yaw
        self._pub_state.publish(m)

    # ── TCP I/O loop (background thread) ──────────────────────────────

    def _io_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            if self._sock is None:
                self._sock = self._try_connect()
                if self._sock is None:
                    time.sleep(self._reconn_s)
                    continue

            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise ConnectionResetError("gimbal closed connection")
                buf.extend(chunk)
            except socket.timeout:
                # Gimbal isn't sending unsolicited state — totally normal.
                # The outbound command timer keeps writing setpoints on a
                # separate timer; we just keep listening.
                continue
            except (OSError, ConnectionResetError) as exc:
                self.get_logger().warning(f"Gimbal recv failed: {exc}")
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                buf.clear()
                continue

            # Drain all complete packets currently in the buffer
            while True:
                consumed, parsed = parse_state_packet(buf)
                if consumed == 0:
                    break
                del buf[:consumed]
                if parsed is not None:
                    self._handle_inbound(parsed)

    def _try_connect(self) -> socket.socket | None:
        try:
            sock = socket.create_connection((self._host, self._port), timeout=3.0)
            sock.settimeout(1.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info(f"Gimbal TCP connected — {self._host}:{self._port}")
            return sock
        except OSError as exc:
            self.get_logger().warning(
                f"Gimbal TCP connect failed ({exc}) — retry in {self._reconn_s:.1f}s"
            )
            return None

    def _handle_inbound(self, parsed: dict):
        # parse_state_packet returns roll/pitch/yaw as signed centi-degrees
        # converted to degrees. Normalise yaw to -180..+180 in case the
        # gimbal ever sends an out-of-range value.
        yaw_deg = parsed["yaw_deg"]
        yaw_norm = ((yaw_deg + 180.0) % 360.0) - 180.0
        with self._lock:
            self._state_roll  = parsed["roll_deg"]
            self._state_pitch = parsed["pitch_deg"]
            self._state_yaw   = yaw_norm
        self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
