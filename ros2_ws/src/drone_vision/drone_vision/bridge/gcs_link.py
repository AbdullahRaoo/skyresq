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
import socket
import time

import rclpy
from rclpy.node import Node

from drone_msgs.msg import TargetWorld

EARTH_R = 6_371_000.0


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

        self._gcs_ip   = self.get_parameter("gcs_ip").value
        self._gcs_port = int(self.get_parameter("gcs_port").value)
        self._radius_m = float(self.get_parameter("cluster_radius_m").value)
        self._upd_s    = float(self.get_parameter("update_interval_s").value)
        self._conf_min = float(self.get_parameter("confidence_min").value)
        mav_port       = self.get_parameter("mavlink_port").value
        mav_baud       = int(self.get_parameter("mavlink_baud").value)

        self._clusters:  dict[str, _Cluster] = {}
        self._last_send: dict[str, float]    = {}
        self._link_ok = True

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

        # Periodic SiK heartbeat and status (1 Hz)
        if self._mav:
            self.create_timer(1.0, self._sik_heartbeat)

        self.get_logger().info(
            f"GCS link ready → {self._gcs_ip}:{self._gcs_port} | "
            f"cluster_r={self._radius_m} m | conf≥{self._conf_min}"
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
        payload = (json.dumps(cluster.to_dict()) + "\n").encode()
        try:
            self._sock.sendto(payload, (self._gcs_ip, self._gcs_port))
            self._link_ok = True
            self._last_send[cluster.id] = time.monotonic()
        except OSError as exc:
            self._link_ok = False
            self.get_logger().warning(f"UDP send failed: {exc}")

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
