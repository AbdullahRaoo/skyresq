#!/usr/bin/env python3
"""
Mock Pi sender — simulates the Pi's UDP feed to the SkyResQ GCS.

Lets the dashboard-side dev test the UDP listener, survivor-marker
rendering, video overlay, and pi_status health panel WITHOUT a real
drone, Pi, or pymavlink installation. Pure Python stdlib.

Packets emitted
---------------
  survivor_cluster   3 fake clusters, one slowly migrating + one stable
  detection_frame    8 Hz, 1-3 bboxes that walk across the frame
  pi_status          1 Hz, alternating healthy / temporarily degraded

Usage
-----
  python3 mock_pi_sender.py
  python3 mock_pi_sender.py --host 127.0.0.1 --port 5005
  python3 mock_pi_sender.py --rate-detect 5 --rate-status 2
  python3 mock_pi_sender.py --types pi_status,survivor_cluster
"""

import argparse
import json
import math
import random
import socket
import time


# ── Defaults — match the canonical drone+GCS configuration ───────────
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005

# Anchor location: roughly Islamabad (matches the lat/lon in the spec)
ANCHOR_LAT = 33.72938
ANCHOR_LON = 73.09312

STREAM_W = 1280
STREAM_H = 720


def _gen_clusters():
    """Generate three plausible survivor clusters."""
    return [
        {
            "id":            "cluster-mock00000001",
            "count":         3,
            "lat":           ANCHOR_LAT + 0.00005,
            "lon":           ANCHOR_LON + 0.00010,
            "confidence":    0.91,
            "first_seen_ms": int(time.time() * 1000) - 12_000,
            "status":        "new",
        },
        {
            "id":            "cluster-mock00000002",
            "count":         1,
            "lat":           ANCHOR_LAT - 0.00012,
            "lon":           ANCHOR_LON - 0.00008,
            "confidence":    0.67,
            "first_seen_ms": int(time.time() * 1000) - 6_000,
            "status":        "new",
        },
        {
            "id":            "cluster-mock00000003",
            "count":         2,
            "lat":           ANCHOR_LAT + 0.00020,
            "lon":           ANCHOR_LON - 0.00005,
            "confidence":    0.78,
            "first_seen_ms": int(time.time() * 1000) - 3_000,
            "status":        "new",
        },
    ]


def _cluster_packet(c, now_ms, n_samples):
    return {
        "type":          "survivor_cluster",
        "id":            c["id"],
        "count":         c["count"],
        "lat":           round(c["lat"], 7),
        "lon":           round(c["lon"], 7),
        "alt":           0.0,
        "confidence":    round(c["confidence"], 3),
        "first_seen_ms": c["first_seen_ms"],
        "last_seen_ms":  now_ms,
        "n_samples":     n_samples,
        "status":        c["status"],
    }


def _detection_packet(now_ms, frame_idx, n_detections):
    """One detection_frame packet with `n_detections` bboxes walking across."""
    dets = []
    for k in range(n_detections):
        phase = (frame_idx * 0.02 + k * 0.6) % (2 * math.pi)
        cx = (STREAM_W // 2) + int(280 * math.sin(phase))
        cy = (STREAM_H // 2) + int(120 * math.cos(phase * 0.7))
        w  = 80 + int(20 * math.sin(phase * 1.3))
        h  = 180 + int(40 * math.cos(phase * 0.9))
        dets.append({
            "bbox":       [cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2],
            "confidence": round(0.70 + 0.25 * abs(math.sin(phase)), 3),
            "class":      "person",
            "cluster_id": None,
        })
    return {
        "type":          "detection_frame",
        "frame_ts_ms":   now_ms,
        "stream_width":  STREAM_W,
        "stream_height": STREAM_H,
        "detections":    dets,
    }


def _status_packet(uptime_s, healthy):
    """Periodically alternates between healthy and 'detector choked' states."""
    return {
        "type":         "pi_status",
        "ts_ms":        int(time.time() * 1000),
        "uptime_s":     uptime_s,
        "cpu_temp_c":   round(54.0 + (10.0 if not healthy else 4.0)
                              + random.uniform(-1.5, 1.5), 1),
        "cpu_load1":    round((0.45 if healthy else 0.92)
                              + random.uniform(-0.05, 0.05), 2),
        "ram_used_mb":  1840 + random.randint(-50, 50),
        "ram_total_mb": 4096,
        "detector": {
            "ok":  healthy,
            "fps": round(8.4 if healthy else 1.1, 1),
        },
        "camera": {
            "ok":  True,
            "fps": round(24.0 + random.uniform(-0.5, 0.5), 1),
        },
        "gimbal": {
            "ok":        True,
            "pitch_deg": -90.0,
            "yaw_deg":   round(math.sin(uptime_s / 8.0) * 15.0, 1),
        },
        "fc_link": {
            "ok":    True,
            "armed": False,
        },
        "gcs_link": {"ok": True},
        "cluster_count": 3,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host",           default=DEFAULT_HOST)
    ap.add_argument("--port",           default=DEFAULT_PORT, type=int)
    ap.add_argument("--rate-detect",    default=8.0, type=float,
                    help="detection_frame Hz")
    ap.add_argument("--rate-status",    default=1.0, type=float,
                    help="pi_status Hz")
    ap.add_argument("--cluster-update", default=2.0, type=float,
                    help="survivor_cluster update interval (s)")
    ap.add_argument("--types",
                    default="survivor_cluster,detection_frame,pi_status",
                    help="comma-separated subset of packet types to emit")
    args = ap.parse_args()

    enabled = set(t.strip() for t in args.types.split(","))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    clusters = _gen_clusters()
    start = time.time()
    frame_idx = 0
    last_cluster = 0.0
    last_status  = 0.0
    last_detect  = 0.0

    print(f"Mock Pi sender → {args.host}:{args.port}")
    print(f"  types     : {sorted(enabled)}")
    print(f"  detect Hz : {args.rate_detect}")
    print(f"  status Hz : {args.rate_status}")
    print(f"  cluster Δs: {args.cluster_update}")
    print()

    try:
        while True:
            now = time.time()
            now_ms = int(now * 1000)
            uptime = int(now - start)

            # detection_frame
            if "detection_frame" in enabled and now - last_detect >= 1.0 / args.rate_detect:
                n = 1 + (frame_idx // 30) % 3      # cycles 1→2→3 detections
                _send(sock, args.host, args.port,
                      _detection_packet(now_ms, frame_idx, n))
                frame_idx += 1
                last_detect = now

            # survivor_cluster
            if "survivor_cluster" in enabled and now - last_cluster >= args.cluster_update:
                for k, c in enumerate(clusters):
                    # First cluster drifts north slowly (simulates re-detection)
                    if k == 0:
                        c["lat"] += 0.0000005
                    n_samples = 1 + uptime * 4
                    _send(sock, args.host, args.port,
                          _cluster_packet(c, now_ms, n_samples))
                last_cluster = now

            # pi_status — alternate healthy/degraded every 20 s to exercise UI
            if "pi_status" in enabled and now - last_status >= 1.0 / args.rate_status:
                healthy = (uptime % 40) < 25
                _send(sock, args.host, args.port,
                      _status_packet(uptime, healthy))
                last_status = now

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopped.")


def _send(sock, host, port, obj):
    payload = (json.dumps(obj) + "\n").encode()
    sock.sendto(payload, (host, port))


if __name__ == "__main__":
    main()
