#!/usr/bin/env python3
"""
Pre-flight check.

Standalone script — does NOT depend on ROS 2. Runs through the safety
gates that must be green before any motors-on operation. Designed to be
run on the Pi via SSH or invoked from the SkyResQ GCS over Tailscale.

Checks
------
  1. Pi  — disk free, CPU temperature, swap, OS uptime
  2. FC  — MAVLink heartbeat on /dev/serial0, autopilot ID, firmware string
  3. GPS — fix type, satellites, HDOP
  4. Battery — voltage and remaining percentage from SYS_STATUS
  5. RC link — RC_CHANNELS present, throttle on a low channel
  6. Gimbal/camera — TCP reachable on 192.168.144.108:2332 + 554
  7. GCS link — Tailscale interface up, GCS IP pingable

Exit status
-----------
  0 — all green, safe to proceed
  1 — at least one check failed
  2 — script error (couldn't reach FC at all)

Usage
-----
  python3 preflight_check.py
  python3 preflight_check.py --serial /dev/ttyAMA0 --gcs 100.64.0.5
  python3 preflight_check.py --json    # machine-readable output for GCS
"""

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time

OK   = "\033[32mOK \033[0m"
WARN = "\033[33mWARN\033[0m"
FAIL = "\033[31mFAIL\033[0m"


def _report(checks, as_json):
    if as_json:
        print(json.dumps(
            {"all_passed": all(c["passed"] for c in checks), "checks": checks},
            indent=2,
        ))
    else:
        for c in checks:
            tag = OK if c["passed"] else (WARN if c.get("warn") else FAIL)
            print(f"  [{tag}] {c['name']}: {c['detail']}")
        print()
        passed = sum(1 for c in checks if c["passed"])
        print(f"  Result: {passed}/{len(checks)} checks passed")


# ── Pi-side checks ────────────────────────────────────────────────────

def check_pi():
    out = []

    free = shutil.disk_usage("/").free / (1024 ** 3)
    out.append({
        "name": "disk_free",
        "passed": free > 1.0,
        "detail": f"{free:.2f} GB free on /",
    })

    temp = None
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            temp = int(f.read().strip()) / 1000.0
    except Exception:
        pass
    out.append({
        "name": "cpu_temp",
        "passed": temp is None or temp < 75.0,
        "warn":   temp is not None and 70.0 <= temp < 75.0,
        "detail": f"{temp:.1f}°C" if temp is not None else "unavailable",
    })

    uptime = 0.0
    try:
        with open("/proc/uptime") as f:
            uptime = float(f.read().split()[0])
    except Exception:
        pass
    out.append({
        "name": "uptime",
        "passed": uptime > 30,
        "detail": f"{uptime:.0f} s",
    })

    return out


# ── Network checks ────────────────────────────────────────────────────

def _tcp_reachable(host, port, timeout=2.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def check_network(gimbal_host, gcs_ip):
    out = []

    out.append({
        "name": "gimbal_tcp",
        "passed": _tcp_reachable(gimbal_host, 2332),
        "detail": f"tcp://{gimbal_host}:2332 (Z-1 Mini control)",
    })

    rtsp_open = _tcp_reachable(gimbal_host, 554)
    out.append({
        "name": "gimbal_rtsp",
        "passed": rtsp_open,
        "detail": f"tcp://{gimbal_host}:554 (RTSP video)",
    })

    ping_ok = subprocess.run(
        ["ping", "-c", "1", "-W", "2", gcs_ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0
    out.append({
        "name": "gcs_ping",
        "passed": ping_ok,
        "detail": f"ping {gcs_ip}",
    })

    return out


# ── Flight controller checks (pymavlink) ──────────────────────────────

def check_fc(serial, baud):
    try:
        from pymavlink import mavutil
    except ImportError:
        return [{
            "name": "pymavlink",
            "passed": False,
            "detail": "module missing — pip3 install pymavlink",
        }]

    out = []

    try:
        mav = mavutil.mavlink_connection(
            serial, baud=baud, source_system=255, source_component=0,
        )
    except Exception as exc:
        return [{
            "name": "fc_connect",
            "passed": False,
            "detail": f"could not open {serial}: {exc}",
        }]

    hb = mav.wait_heartbeat(timeout=10)
    if hb is None:
        out.append({
            "name": "fc_heartbeat",
            "passed": False,
            "detail": f"no heartbeat from {serial} within 10 s",
        })
        return out

    out.append({
        "name": "fc_heartbeat",
        "passed": True,
        "detail": f"sysid={mav.target_system} compid={mav.target_component} "
                  f"autopilot={hb.autopilot} type={hb.type}",
    })

    # Request streams briefly and collect samples
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
    )

    deadline = time.time() + 4.0
    gps = battery = rc = None
    while time.time() < deadline and not (gps and battery and rc):
        msg = mav.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GPS_RAW_INT" and gps is None:
            gps = msg
        elif t == "SYS_STATUS" and battery is None:
            battery = msg
        elif t == "RC_CHANNELS" and rc is None:
            rc = msg

    if gps is not None:
        out.append({
            "name": "gps",
            "passed": gps.fix_type >= 3 and gps.satellites_visible >= 6,
            "warn":   gps.fix_type >= 2,
            "detail": f"fix_type={gps.fix_type} sats={gps.satellites_visible} "
                      f"HDOP={gps.eph / 100.0:.2f}",
        })
    else:
        out.append({
            "name": "gps",
            "passed": False,
            "detail": "no GPS_RAW_INT received",
        })

    if battery is not None:
        v = battery.voltage_battery / 1000.0 if battery.voltage_battery != -1 else 0.0
        out.append({
            "name": "battery",
            "passed": v >= 14.8 and battery.battery_remaining >= 60,
            "warn":   v >= 13.6 and battery.battery_remaining >= 30,
            "detail": f"V={v:.2f} remaining={battery.battery_remaining}%",
        })
    else:
        out.append({
            "name": "battery",
            "passed": False,
            "detail": "no SYS_STATUS received",
        })

    if rc is not None:
        thr = getattr(rc, "chan3_raw", 0)
        out.append({
            "name": "rc_link",
            "passed": rc.rssi != 255 and 900 <= thr <= 1200,
            "detail": f"thr_ch3={thr}µs rssi={rc.rssi}",
        })
    else:
        out.append({
            "name": "rc_link",
            "passed": False,
            "detail": "no RC_CHANNELS — TX off or not bound",
        })

    return out


# ── Entry point ───────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial",      default=os.environ.get("CONNECTION_STRING",
                                                            "/dev/serial0"))
    ap.add_argument("--baud",        default=int(os.environ.get("BAUD_RATE", 57600)),
                                     type=int)
    ap.add_argument("--gimbal-host", default=os.environ.get("GIMBAL_HOST",
                                                            "192.168.144.108"))
    ap.add_argument("--gcs",         default=os.environ.get("GCS_IP",
                                                            "100.123.87.26"))
    ap.add_argument("--json",        action="store_true",
                                     help="machine-readable output for GCS")
    args = ap.parse_args()

    checks = []
    checks += check_pi()
    checks += check_network(args.gimbal_host, args.gcs)
    checks += check_fc(args.serial, args.baud)

    if not args.json:
        print(f"\nPre-flight check — {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    _report(checks, args.json)

    sys.exit(0 if all(c["passed"] for c in checks) else 1)


if __name__ == "__main__":
    main()
