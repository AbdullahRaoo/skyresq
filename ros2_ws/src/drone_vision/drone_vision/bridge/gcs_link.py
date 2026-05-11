#!/usr/bin/env python3
"""
GCS Link Node.

Subscribes to /target/world (drone_msgs/TargetWorld) and forwards
survivor detections to the SkyResQ GCS over UDP as newline-delimited JSON.

Detection clustering
--------------------
Consecutive detections within cluster_radius_m of an existing cluster are
merged into it (running-mean position, max confidence).  A new cluster is
created when no existing one is within range.  The GCS receives a
survivor_cluster JSON packet on cluster creation and on every update that
is at least update_interval_s after the previous send for that cluster.

UDP protocol (matches §8.3 of SKYRESQ_GCS_CHANGES.md)
------------------------------------------------------
  Destination: gcs_ip:gcs_port (default 100.123.87.26:5005)
  Format:      one JSON object per datagram, newline-terminated
  Loss:        UDP — acceptable; GCS handles gaps gracefully

Packet types sent
-----------------
  survivor_cluster   on cluster create + every update_interval_s
  detection_frame    on every Detection2DArray (~detector rate)
  pi_status          1 Hz heartbeat with companion-computer health

SiK fallback
------------
If a pymavlink connection is provided via the mavlink_port / mavlink_baud
parameters, the node also sends NAMED_VALUE_INT messages to the FC so the
GCS can see basic status even when 4G/Tailscale is down:
  cluster_count   total clusters created this mission
  link_4g_ok      1 if last UDP send succeeded, 0 otherwise

Parameters
----------
gcs_ip             str    100.123.87.26   SkyResQ GCS Tailscale IP
gcs_port           int    5005            UDP receive port on the GCS
cluster_radius_m   float  5.0             merge radius for detections
update_interval_s  float  2.0             min seconds between updates for same cluster
confidence_min     float  0.50            ignore detections below this threshold
mavlink_port       str    ""              leave empty to skip SiK fallback
mavlink_baud       int    57600
"""

import hashlib
import json
import math
import os
import socket
import time

import rclpy
from rclpy.node import Node

from drone_msgs.msg import TargetWorld
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

EARTH_R = 6_371_000.0


# ── Pi-side health helpers ───────────────────────────────────────────

def _read_cpu_temp_c() -> float | None:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def _read_meminfo_mb() -> tuple[int, int]:
    """Return (used_mb, total_mb). (0, 0) if unavailable."""
    try:
        total = available = 0
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":")
                v = v.strip().split()
                if not v:
                    continue
                kb = int(v[0])
                if k == "MemTotal":
                    total = kb
                elif k == "MemAvailable":
                    available = kb
        if total:
            return ((total - available) // 1024, total // 1024)
    except (OSError, ValueError):
        pass
    return (0, 0)


def _read_load1() -> float:
    try:
        return os.getloadavg()[0]
    except (OSError, AttributeError):
        return 0.0


class _RateCounter:
    """Counts messages seen in a rolling 2 s window for FPS estimation."""

    __slots__ = ("_stamps",)

    def __init__(self) -> None:
        self._stamps: list[float] = []

    def tick(self) -> None:
        now = time.monotonic()
        self._stamps.append(now)
        cutoff = now - 2.0
        # Trim from the front — list is already monotonically increasing
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.pop(0)

    def fps(self) -> float:
        return len(self._stamps) / 2.0 if self._stamps else 0.0

    @property
    def last_seen(self) -> float:
        return self._stamps[-1] if self._stamps else 0.0


def _haversine_m(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1))
         * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return 2 * EARTH_R * math.asin(math.sqrt(a))


class _Cluster:
    """Running-mean survivor cluster."""

    def __init__(self, lat, lon, confidence, ts_ms):
        tag = f"{ts_ms}-{lat:.4f}-{lon:.4f}"
        self.id            = "cluster-" + hashlib.md5(tag.encode()).hexdigest()[:12]
        self.lat           = lat
        self.lon           = lon
        self.confidence    = confidence
        self.count         = 1
        self.n_samples     = 1
        self.first_seen_ms = ts_ms
        self.last_seen_ms  = ts_ms
        self.status        = "new"

    def update(self, lat, lon, confidence, ts_ms):
        n = self.n_samples
        self.lat        = (self.lat * n + lat) / (n + 1)
        self.lon        = (self.lon * n + lon) / (n + 1)
        self.confidence = max(self.confidence, confidence)
        self.last_seen_ms = ts_ms
        self.n_samples   += 1

    def to_dict(self):
        return {
            "type":          "survivor_cluster",
            "id":            self.id,
            "count":         self.count,
            "lat":           round(self.lat, 7),
            "lon":           round(self.lon, 7),
            "alt":           0.0,
            "confidence":    round(self.confidence, 3),
            "first_seen_ms": self.first_seen_ms,
            "last_seen_ms":  self.last_seen_ms,
            "n_samples":     self.n_samples,
            "status":        self.status,
        }


class GcsLinkNode(Node):

    def __init__(self):
        super().__init__("gcs_link")

        self.declare_parameter("gcs_ip",            "100.123.87.26")
        self.declare_parameter("gcs_port",          5005)
        self.declare_parameter("cluster_radius_m",  5.0)
        self.declare_parameter("update_interval_s", 2.0)
        self.declare_parameter("confidence_min",    0.50)
        self.declare_parameter("mavlink_port",      "")
        self.declare_parameter("mavlink_baud",      57600)
        self.declare_parameter("stream_width",      1280)
        self.declare_parameter("stream_height",     720)
        self.declare_parameter("status_period_s",   1.0)

        self._gcs_ip   = self.get_parameter("gcs_ip").value
        self._gcs_port = int(self.get_parameter("gcs_port").value)
        self._radius_m = float(self.get_parameter("cluster_radius_m").value)
        self._upd_s    = float(self.get_parameter("update_interval_s").value)
        self._conf_min = float(self.get_parameter("confidence_min").value)
        mav_port       = self.get_parameter("mavlink_port").value
        mav_baud       = int(self.get_parameter("mavlink_baud").value)
        self._stream_w = int(self.get_parameter("stream_width").value)
        self._stream_h = int(self.get_parameter("stream_height").value)
        status_period  = float(self.get_parameter("status_period_s").value)

        self._clusters:  dict[str, _Cluster] = {}
        self._last_send: dict[str, float]    = {}
        self._link_ok = True
        self._start_t = time.monotonic()

        # Health trackers — populated by the *_seen subscriptions
        self._detector_rate = _RateCounter()
        self._camera_rate   = _RateCounter()
        self._gimbal_rate   = _RateCounter()
        self._fc_rate       = _RateCounter()
        self._gimbal_pitch  = 0.0
        self._gimbal_yaw    = 0.0
        self._fc_armed      = False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Optional SiK fallback via pymavlink
        self._mav = None
        if mav_port:
            try:
                from pymavlink import mavutil
                self._mav = mavutil.mavlink_connection(
                    mav_port, baud=mav_baud,
                    source_system=255, source_component=191,
                )
                self._mav.wait_heartbeat(timeout=10)
                self.get_logger().info(f"SiK fallback active on {mav_port}")
            except Exception as exc:
                self.get_logger().warning(f"SiK fallback unavailable: {exc}")
                self._mav = None

        self.create_subscription(TargetWorld, "/target/world", self._on_target, 10)
        self.create_subscription(
            Detection2DArray, "/detections", self._on_detection_frame, 10
        )
        self.create_subscription(Image, "/drone/camera_raw", self._on_camera, 1)
        self.create_subscription(Vector3Stamped, "/gimbal/state", self._on_gimbal, 10)
        self.create_subscription(Bool, "/vehicle/armed", self._on_armed, 10)

        # 1 Hz Pi-status heartbeat (operator sees companion health)
        self.create_timer(status_period, self._send_pi_status)

        # Periodic SiK heartbeat and status (1 Hz)
        if self._mav:
            self.create_timer(1.0, self._sik_heartbeat)

        self.get_logger().info(
            f"GCS link ready → {self._gcs_ip}:{self._gcs_port} | "
            f"cluster_r={self._radius_m} m | conf≥{self._conf_min} | "
            f"stream={self._stream_w}x{self._stream_h}"
        )

    # ── Detection callback ─────────────────────────────────────────────

    def _on_target(self, msg: TargetWorld):
        if float(msg.confidence) < self._conf_min:
            return

        geo = msg.position_geo
        if geo.status.status < 0:
            return  # no GPS fix — can't geo-locate

        lat  = geo.latitude
        lon  = geo.longitude
        conf = float(msg.confidence)
        ts_ms = int(self.get_clock().now().nanoseconds / 1_000_000)

        cluster = self._nearest_cluster(lat, lon)
        if cluster is None:
            cluster = _Cluster(lat, lon, conf, ts_ms)
            self._clusters[cluster.id] = cluster
            self.get_logger().info(
                f"New cluster {cluster.id[:16]}  ({lat:.6f}, {lon:.6f})  "
                f"conf={conf:.2f}"
            )
            self._send_cluster(cluster)
        else:
            cluster.update(lat, lon, conf, ts_ms)
            now = time.monotonic()
            if now - self._last_send.get(cluster.id, 0.0) >= self._upd_s:
                self._send_cluster(cluster)

    # ── detection_frame forwarding ────────────────────────────────────

    def _on_detection_frame(self, msg: Detection2DArray):
        """Forward raw YOLO bboxes to the GCS for live video overlay."""
        self._detector_rate.tick()
        if not msg.detections:
            return

        det_list = []
        for d in msg.detections:
            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            w  = d.bbox.size_x
            h  = d.bbox.size_y
            x1 = max(0.0, cx - w / 2.0)
            y1 = max(0.0, cy - h / 2.0)
            x2 = cx + w / 2.0
            y2 = cy + h / 2.0

            conf = 0.0
            cls  = "person"
            if d.results:
                conf = float(d.results[0].hypothesis.score)
                cls  = d.results[0].hypothesis.class_id or "person"

            det_list.append({
                "bbox":       [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                "confidence": round(conf, 3),
                "class":      cls,
                "cluster_id": None,
            })

        stamp_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000
        packet = {
            "type":          "detection_frame",
            "frame_ts_ms":   stamp_ms,
            "stream_width":  self._stream_w,
            "stream_height": self._stream_h,
            "detections":    det_list,
        }
        self._send_json(packet)

    # ── Health tracker callbacks ──────────────────────────────────────

    def _on_camera(self, _msg: Image):
        # Use the message to learn the actual stream size on first arrival
        if (_msg.width and _msg.height
                and (_msg.width != self._stream_w or _msg.height != self._stream_h)):
            self._stream_w = int(_msg.width)
            self._stream_h = int(_msg.height)
            self.get_logger().info(
                f"stream size learnt from /drone/camera_raw: "
                f"{self._stream_w}x{self._stream_h}"
            )
        self._camera_rate.tick()

    def _on_gimbal(self, msg: Vector3Stamped):
        self._gimbal_rate.tick()
        self._gimbal_pitch = float(msg.vector.y)
        self._gimbal_yaw   = float(msg.vector.z)

    def _on_armed(self, msg: Bool):
        self._fc_rate.tick()
        self._fc_armed = bool(msg.data)

    # ── pi_status (1 Hz) ──────────────────────────────────────────────

    def _send_pi_status(self):
        now    = time.monotonic()
        uptime = int(now - self._start_t)

        ram_used, ram_total = _read_meminfo_mb()

        def _fresh(counter: _RateCounter, max_age_s: float) -> bool:
            return counter.last_seen > 0 and (now - counter.last_seen) <= max_age_s

        packet = {
            "type":          "pi_status",
            "ts_ms":         int(time.time() * 1000),
            "uptime_s":      uptime,
            "cpu_temp_c":    _read_cpu_temp_c(),
            "cpu_load1":     round(_read_load1(), 2),
            "ram_used_mb":   ram_used,
            "ram_total_mb": ram_total,
            "detector": {
                "ok":          _fresh(self._detector_rate, 5.0),
                "fps":         round(self._detector_rate.fps(), 1),
            },
            "camera": {
                "ok":          _fresh(self._camera_rate, 2.0),
                "fps":         round(self._camera_rate.fps(), 1),
            },
            "gimbal": {
                "ok":          _fresh(self._gimbal_rate, 2.0),
                "pitch_deg":   round(self._gimbal_pitch, 1),
                "yaw_deg":     round(self._gimbal_yaw, 1),
            },
            "fc_link": {
                "ok":          _fresh(self._fc_rate, 3.0),
                "armed":       self._fc_armed,
            },
            "gcs_link": {
                "ok":          self._link_ok,
            },
            "cluster_count": len(self._clusters),
        }
        self._send_json(packet)

    # ── UDP send (shared) ─────────────────────────────────────────────

    def _send_json(self, obj: dict) -> None:
        payload = (json.dumps(obj) + "\n").encode()
        try:
            self._sock.sendto(payload, (self._gcs_ip, self._gcs_port))
            self._link_ok = True
        except OSError as exc:
            self._link_ok = False
            self.get_logger().debug(f"UDP send failed: {exc}")

    # ── Clustering ─────────────────────────────────────────────────────

    def _nearest_cluster(self, lat, lon):
        best_dist = self._radius_m
        best      = None
        for c in self._clusters.values():
            d = _haversine_m(c.lat, c.lon, lat, lon)
            if d < best_dist:
                best_dist = d
                best      = c
        return best

    # ── UDP send ───────────────────────────────────────────────────────

    def _send_cluster(self, cluster: _Cluster):
        self._send_json(cluster.to_dict())
        if self._link_ok:
            self._last_send[cluster.id] = time.monotonic()

    # ── SiK fallback (optional 1 Hz) ──────────────────────────────────

    def _sik_heartbeat(self):
        if not self._mav:
            return
        try:
            from pymavlink import mavutil as mu
            ts = int(time.time() * 1000) & 0xFFFF_FFFF

            self._mav.mav.heartbeat_send(
                mu.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mu.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            self._mav.mav.named_value_int_send(
                ts, b"cluster_cnt", len(self._clusters)
            )
            self._mav.mav.named_value_int_send(
                ts, b"link_4g_ok\x00", int(self._link_ok)
            )
        except Exception as exc:
            self.get_logger().debug(f"SiK send error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GcsLinkNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
