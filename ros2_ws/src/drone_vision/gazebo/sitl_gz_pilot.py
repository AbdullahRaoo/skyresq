#!/usr/bin/env python3
"""Gazebo-SITL pilot: ARMING_CHECK=0, FENCE=0, GUIDED arm, takeoff 20m, hold."""
import math, sys, time
from pymavlink import mavutil
def L(*a): print(*a, flush=True)
HOME = (-35.363262, 149.165237); ALT = 20.0
m = mavutil.mavlink_connection('tcp:127.0.0.1:5762'); m.wait_heartbeat()
m.target_component = 1
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
L("hb ok")

def sp(n, v):
    m.mav.param_set_send(m.target_system, m.target_component, n.encode(),
                         float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

sp('ARMING_CHECK', 0); sp('FENCE_ENABLE', 0); time.sleep(2)

t = time.time(); fix = 0
while time.time() - t < 60:
    g = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
    if g: fix = g.fix_type
    if fix >= 3: break
L(f"GPS fix={fix}"); time.sleep(8)
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    m.mode_mapping()['GUIDED']); time.sleep(2)
for force in (False, True, True, True):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,
        21196 if force else 0, 0, 0, 0, 0, 0)
    t = time.time()
    while time.time() - t < 6:
        m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if m.motors_armed(): break
    if m.motors_armed(): L(f"ARMED (force={force})"); break
if not m.motors_armed(): sys.exit("ARM FAILED")
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, ALT)
L(f"takeoff {ALT}m")
t = time.time(); rel = 0
while time.time() - t < 40:
    a = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if a:
        rel = a.relative_alt / 1000.0
        if rel >= ALT - 1.5: break
L(f"AIRBORNE alt={rel:.1f}m")

t = time.time()
while time.time() - t < 240:
    a = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if a and int(time.time() - t) % 5 == 0:
        dn = (a.lat / 1e7 - HOME[0]) * 111320
        de = (a.lon / 1e7 - HOME[1]) * 111320 * math.cos(math.radians(HOME[0]))
        L(f"t={int(time.time()-t)}s alt={a.relative_alt/1000:.1f}m N={dn:+.1f} E={de:+.1f} armed={m.motors_armed()}")
