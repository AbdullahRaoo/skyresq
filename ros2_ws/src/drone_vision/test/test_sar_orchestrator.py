#!/usr/bin/env python3
"""
SAR orchestrator end-to-end integration test.

Drives the real sar_orchestrator node through its full state machine using
mocked ROS inputs in an isolated ROS_DOMAIN_ID, so it needs no GPS, no FC,
and no hardware. Asserts every transition and every emitted intent.

Run on the Pi:
    ROS_DOMAIN_ID=42 python3 test_sar_orchestrator.py

Exit code 0 = all checks passed.
"""
import os
import subprocess
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, String, Empty
from sensor_msgs.msg import NavSatFix
from drone_msgs.msg import TargetWorld, MissionState

LATCH = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

# Test fixture coordinates (Islamabad-ish; values are arbitrary but realistic)
DRONE_LAT, DRONE_LON = 33.684000, 73.047900
SURV_LAT,  SURV_LON  = 33.684300, 73.047900   # ~33 m north of drone
HOME_LAT,  HOME_LON  = 33.683900, 73.047900


class Tester(Node):
    def __init__(self):
        super().__init__("sar_test_driver")
        self.states: list[str] = []
        self.fly_to: list[tuple] = []
        self.set_modes: list[str] = []
        self.payload_cmds: list[str] = []
        self.rtl_count = 0

        self.create_subscription(MissionState, "/mission/state", self._on_state, 10)
        self.create_subscription(NavSatFix, "/mission/fly_to", self._on_flyto, 10)
        self.create_subscription(String, "/mission/set_mode", self._on_setmode, 10)
        self.create_subscription(String, "/payload/cmd", self._on_payload, 10)
        self.create_subscription(Empty, "/mission/cmd_rtl", self._on_rtl, 10)

        self.pub_enable = self.create_publisher(Bool, "/mission/enable", LATCH)
        self.pub_home = self.create_publisher(NavSatFix, "/vehicle/home", LATCH)
        self.pub_armed = self.create_publisher(Bool, "/vehicle/armed", 10)
        self.pub_mode = self.create_publisher(String, "/vehicle/mode", 10)
        self.pub_gps = self.create_publisher(NavSatFix, "/vehicle/gps", 10)
        self.pub_target = self.create_publisher(TargetWorld, "/target/world", 10)
        self.pub_pstate = self.create_publisher(Bool, "/payload/state", LATCH)

        self._gps = (DRONE_LAT, DRONE_LON)
        self._armed = False
        self._mode = "STABILIZE"
        self._emit_target = False
        self._payload_open = False
        self.create_timer(0.1, self._stream)  # 10 Hz mock telemetry

    def _on_state(self, m: MissionState):
        if not self.states or self.states[-1] != m.state:
            self.states.append(m.state)
            print(f"  [state] -> {m.state} ({m.sub_state})")

    def _on_flyto(self, m: NavSatFix):
        self.fly_to.append((m.latitude, m.longitude, m.altitude))

    def _on_setmode(self, m: String):
        self.set_modes.append(m.data)
        print(f"  [set_mode] {m.data}")

    def _on_payload(self, m: String):
        self.payload_cmds.append(m.data)
        print(f"  [payload_cmd] {m.data}")
        if m.data == "open":
            self._payload_open = True  # simulate servo ack

    def _on_rtl(self, _m: Empty):
        self.rtl_count += 1
        print("  [cmd_rtl] received")

    def _stream(self):
        b = Bool(); b.data = self._armed; self.pub_armed.publish(b)
        s = String(); s.data = self._mode; self.pub_mode.publish(s)
        g = NavSatFix(); g.latitude, g.longitude = self._gps
        g.altitude = 488.0; self.pub_gps.publish(g)
        ps = Bool(); ps.data = self._payload_open; self.pub_pstate.publish(ps)
        if self._emit_target:
            t = TargetWorld()
            t.header.stamp = self.get_clock().now().to_msg()
            t.position_geo.latitude = SURV_LAT
            t.position_geo.longitude = SURV_LON
            t.confidence = 0.85
            t.source = TargetWorld.SOURCE_GEO_LOCK
            self.pub_target.publish(t)

    # helpers
    def set_home(self):
        h = NavSatFix(); h.latitude, h.longitude = HOME_LAT, HOME_LON
        h.altitude = 488.0; self.pub_home.publish(h)

    def enable(self, v: bool):
        b = Bool(); b.data = v; self.pub_enable.publish(b)


def wait_for(cond, timeout, desc):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            print(f"PASS: {desc} ({time.time()-t0:.1f}s)")
            return True
        time.sleep(0.1)
    print(f"FAIL: {desc} (timeout {timeout}s)")
    return False


def main():
    env = dict(os.environ)
    proc = subprocess.Popen(
        ["bash", "-c",
         "source /home/pi/Drone/ops/env.sh && exec ros2 run drone_vision sar_orchestrator "
         "--ros-args -p hold_time_s:=1.0 -p drop_hold_s:=2.0 -p approach_tol_m:=5.0 "
         "-p rtl_max_s:=20.0 -p mode_abort_grace_s:=1.5"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    rclpy.init()
    t = Tester()
    spin = threading.Thread(target=rclpy.spin, args=(t,), daemon=True)
    spin.start()

    results = []
    try:
        time.sleep(3.0)  # let orchestrator come up
        t.set_home()
        t.enable(True)

        # 1. IDLE present
        results.append(wait_for(lambda: "IDLE" in t.states, 8, "orchestrator boots to IDLE"))

        # 2. arm + GUIDED -> SEARCH
        t._armed = True
        t._mode = "GUIDED"
        results.append(wait_for(lambda: "SEARCH" in t.states, 8, "IDLE -> SEARCH on armed+GUIDED"))

        # 3. emit target -> DETECTION_HOLD -> APPROACH
        t._emit_target = True
        results.append(wait_for(lambda: "DETECTION_HOLD" in t.states, 6, "SEARCH -> DETECTION_HOLD on target"))
        results.append(wait_for(lambda: "APPROACH" in t.states, 6, "DETECTION_HOLD -> APPROACH after hold"))

        # 4. set_mode GUIDED emitted, fly_to emitted
        results.append(wait_for(lambda: "GUIDED" in t.set_modes, 3, "set_mode GUIDED emitted on commit"))
        results.append(wait_for(lambda: len(t.fly_to) > 0, 3, "fly_to setpoint emitted"))

        # 5. simulate arrival at drop point -> DROP
        if t.fly_to:
            dp_lat, dp_lon, _ = t.fly_to[-1]
            t._gps = (dp_lat, dp_lon)
        results.append(wait_for(lambda: "DROP" in t.states, 6, "APPROACH -> DROP on arrival"))

        # 6. payload open cmd, then DROP_HOLD, then close + RTL
        results.append(wait_for(lambda: "open" in t.payload_cmds, 4, "payload OPEN commanded"))
        results.append(wait_for(lambda: "DROP_HOLD" in t.states, 4, "DROP -> DROP_HOLD on servo ack"))
        results.append(wait_for(lambda: "close" in t.payload_cmds, 6, "payload CLOSE commanded"))
        results.append(wait_for(lambda: t.rtl_count > 0, 4, "RTL commanded"))
        results.append(wait_for(lambda: "RTL" in t.states, 4, "DROP_HOLD -> RTL"))

        # 7. simulate home arrival -> DONE
        t._gps = (HOME_LAT, HOME_LON)
        results.append(wait_for(lambda: "DONE" in t.states, 8, "RTL -> DONE on home arrival"))

        # 8. kill-switch test: re-run to SEARCH then disengage
        t.enable(False)
        time.sleep(2.0)
        # state should be IDLE again
        results.append(wait_for(lambda: t.states[-1] == "IDLE", 5, "disengage returns to IDLE"))

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        rclpy.shutdown()

    npass = sum(1 for r in results if r)
    print(f"\n=== {npass}/{len(results)} checks passed ===")
    print(f"state sequence: {' -> '.join(t.states)}")
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
