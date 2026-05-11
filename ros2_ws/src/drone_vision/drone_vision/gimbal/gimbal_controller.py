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

XFRobot framing (preliminary — verify against AP_Mount_XFRobot.cpp on first bench test)
--------------------------------------------------------------------------------------
  Out: [0xA8, 0xE5, 0x02, len_lo, len_hi, order, ...payload..., crc_lo, crc_hi]
  In:  [0x8A, 0x5E, 0x02, len_lo, len_hi, order, ...payload..., crc_lo, crc_hi]
  Order codes:
    0x14  EULER_ANGLE_CONTROL    (pitch, yaw, roll — int16 centi-deg, little-endian)
    0x10  ANGLE
    0x12  HEAD_FOLLOW
    0x17  TRACK
    0x1A  CLICK_TO_AIM
    0x20  SHUTTER
    0x21  RECORD
    0x25  ZOOM_RATE
    0x75  TARGET_DETECTION_TOGGLE
  CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xor-out)
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

ORDER_EULER_ANGLE_CONTROL = 0x14
ORDER_TRACK               = 0x17

# CRC-16/CCITT-FALSE
_CRC_POLY = 0x1021
_CRC_INIT = 0xFFFF


def _crc16_ccitt(data: bytes) -> int:
    crc = _CRC_INIT
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC_POLY) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_euler_angle_packet(pitch_deg: float, yaw_deg: float, roll_deg: float = 0.0) -> bytes:
    """Build an EULER_ANGLE_CONTROL (0x14) command packet."""
    payload = struct.pack(
        "<hhh",
        int(round(pitch_deg * 100)),
        int(round(yaw_deg   * 100)),
        int(round(roll_deg  * 100)),
    )
    body = (
        bytes([PROTO_VERSION])
        + struct.pack("<H", 1 + len(payload))   # length = order byte + payload
        + bytes([ORDER_EULER_ANGLE_CONTROL])
        + payload
    )
    crc = _crc16_ccitt(body)
    return HDR_OUT + body + struct.pack("<H", crc)


def parse_state_packet(buf: bytearray):
    """
    Try to extract one inbound packet from a streaming buffer.

    Returns (consumed_bytes, parsed_dict_or_None). The parsed dict carries
    {"order": int, "payload": bytes} when a full valid packet is found.
    Bytes before HDR_IN are discarded. Returns (0, None) if not enough data.
    """
    idx = buf.find(HDR_IN)
    if idx < 0:
        # No header in buffer — drop everything except the last byte
        # (which might be the first byte of a header at the boundary)
        return max(0, len(buf) - 1), None
    if idx > 0:
        # Discard pre-header garbage
        return idx, None

    # We have HDR_IN at offset 0; need at least 5 header bytes (header+ver+len)
    if len(buf) < 5:
        return 0, None

    length = struct.unpack_from("<H", buf, 3)[0]   # length covers order + payload
    total  = 2 + 1 + 2 + length + 2                # hdr + ver + len + body + crc
    if len(buf) < total:
        return 0, None

    body = bytes(buf[2:2 + 1 + 2 + length])
    crc_pkt = struct.unpack_from("<H", buf, 2 + 1 + 2 + length)[0]
    if _crc16_ccitt(body) != crc_pkt:
        # Bad CRC — drop the header and keep scanning
        return 1, None

    order   = body[3]
    payload = body[4:]
    return total, {"order": order, "payload": payload}


# ── Node ──────────────────────────────────────────────────────────────

class GimbalControllerNode(Node):

    def __init__(self):
        super().__init__("gimbal_controller")

        self.declare_parameter("gimbal_host",      "192.168.144.108")
        self.declare_parameter("gimbal_port",      2332)
        self.declare_parameter("command_rate_hz",  50.0)
        self.declare_parameter("state_rate_hz",    20.0)
        self.declare_parameter("backend",          "tcp")
        self.declare_parameter("pixel_gain_yaw",   30.0)
        self.declare_parameter("pixel_gain_pitch", 20.0)
        self.declare_parameter("yaw_min_deg",      -180.0)
        self.declare_parameter("yaw_max_deg",      180.0)
        self.declare_parameter("pitch_min_deg",    -90.0)
        self.declare_parameter("pitch_max_deg",    30.0)
        self.declare_parameter("reconnect_s",      2.0)

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
        self._target_pitch = -90.0   # start nadir
        self._target_yaw   = 0.0
        self._state_pitch  = -90.0
        self._state_yaw    = 0.0
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
            sock.sendall(build_euler_angle_packet(pitch, yaw))
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
        order   = parsed["order"]
        payload = parsed["payload"]

        # Heuristic state extraction: first three int16s as pitch/yaw/roll
        # in centi-degrees. Update once the real packet layout is verified
        # against AP_Mount_XFRobot.cpp.
        if len(payload) >= 6:
            try:
                pitch, yaw, roll = struct.unpack_from("<hhh", payload, 0)
                with self._lock:
                    self._state_pitch = pitch / 100.0
                    self._state_yaw   = yaw   / 100.0
                    self._state_roll  = roll  / 100.0
                self._publish_state()
            except struct.error:
                pass


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
