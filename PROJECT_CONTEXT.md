# SkyResQ Drone — Project Context & Status

Autonomous Search-and-Rescue drone: detects a person from the air, geo-locates
them, flies to them, drops a rescue payload, and returns to launch — with the
operator able to take over at any instant from the RC transmitter.

> This document is the single source of truth for system state. Updated
> 2026-05-14.

---

## 1. Architecture (current — hardware, not SITL)

The project pivoted off the early PX4/Gazebo SITL work. Production stack:

```
GROUND (operator laptop)
  └── SkyResQ Dashboard  (Electron + Next.js)   repo: AbdullahRaoo/sky-resq-dashboard
        ├── MAVLink over SiK USB radio  (COM/ttyUSB, 57600)   ← command + telemetry
        ├── MAVLink mirror over UDP 14550 (Tailscale/LAN)     ← telemetry, secondary
        ├── SAR JSON over UDP 5005                             ← survivor/detection/pi_status
        └── Video: WHEP/WebRTC (8889) or HLS (8888) from Pi mediamtx

AIR
  ├── Cube Black — ArduCopter 4.6.3
  │     ├── TELEM1 → SiK radio → ground
  │     └── TELEM2 → /dev/serial0 → Raspberry Pi 4 (companion)
  ├── XF Z-1 Mini gimbal  (192.168.144.108)
  │     ├── RTSP video :554
  │     └── XFRobot TCP control :2332
  ├── Rescue payload servo on RPi GPIO 16 / pin 36 (hobby servo, lgpio PWM)
  └── Raspberry Pi 4 (Debian Trixie, ROS 2 Humble via RoboStack/conda)
        └── 9 ROS nodes (systemd-managed, auto-start on boot)
```

Comm-link doctrine: **SiK is the always-on failsafe link**; Tailscale/4G is
secondary (drops to DERP relay under symmetric NAT — high latency). The demo
runs both ground + Pi on the same Blaze Wi-Fi for LAN-direct, low-latency
video.

## 2. ROS 2 nodes (package `drone_vision`)

| Node                | Role                                                                 |
| ------------------- | -------------------------------------------------------------------- |
| `mavlink_bridge`    | Pi↔FC. Telemetry → ROS topics; mirrors MAVLink to GCS; routes GCS→Pi `MAV_CMD_USER_1/2`; translates orchestrator intents to FC commands. Pi identifies as **sysid=2, compid=191**. |
| `rtsp_camera`       | Pulls the local mediamtx relay → `/drone/camera_raw`.                 |
| `person_detector`   | YOLO26-Nano ncnn ARM-NEON @ 320×320, `best_by=area`. Publishes `/target_position`, `/detections`. |
| `gimbal_controller` | XFRobot binary protocol over TCP. Per-event pixel-error integration + slew smoothing. Single-instance flock; graceful TCP close. |
| `visual_servo`      | One-shot passthrough `/target_position` → `/gimbal/cmd/look_at_pixel`. |
| `geo_localiser`     | Gimbal angle + drone GPS + attitude → survivor world coords (`/target/world`). |
| `payload_servo`     | lgpio PWM on BCM16. `/payload/cmd` (open/close/toggle), `/payload/state`. PWM 544/2400 µs (Arduino-equivalent). |
| `gcs_link`          | Serialises `/target/world`, `/detections`, `/mission/state` → SAR JSON UDP to dashboard. |
| `sar_orchestrator`  | Autonomy state machine (opt-in, not auto-started). IDLE→SEARCH→DETECTION_HOLD→APPROACH→DROP→DROP_HOLD→RTL→DONE. |

## 3. Autonomous mission flow

1. Operator (RC TX) arms + sets a flight mode. Dashboard ARM/MODE are
   convenience only — **stick/TX has priority** (ArduPilot `ARMING_RUDDER`).
2. Dashboard: draw search polygon (each vertex enforced **≤400 m** from drone;
   refused entirely if no GPS fix). Lawnmower grid auto-generated, uploaded.
3. Operator flips TX to GUIDED (or dashboard ENGAGE — requires position).
4. Dashboard "🤖 Engage SAR Autonomy" → `MAV_CMD_USER_2` → Pi `/mission/enable`.
5. `sar_orchestrator`: on armed+GUIDED → SEARCH. On a confident detection
   (`/target/world` conf ≥ 0.45) → DETECTION_HOLD (1 s confirm) → APPROACH.
6. APPROACH: computes drop point **4 m from survivor on the drone side**,
   publishes `/mission/set_mode GUIDED` + `/mission/fly_to`. `mavlink_bridge`
   sends `SET_POSITION_TARGET_GLOBAL_INT` (type_mask 0xDF8, no FORCE_SET).
7. On arrival (≤2.5 m): DROP → servo OPEN → DROP_HOLD (3 s) → servo CLOSE +
   `/mission/cmd_rtl` (`MAV_CMD_NAV_RETURN_TO_LAUNCH`) → RTL → DONE.
8. **Safety:** any non-GUIDED mode for >2 s during APPROACH/DROP aborts to
   IDLE (pilot took over). `/mission/enable=false` is a hard kill-switch.
   RTL has a 180 s liveness timeout so it never hangs.

## 4. Payload command paths (both verified working)

- **Primary — SiK:** dashboard → SiK → FC TELEM1 → MAVLink router → TELEM2 →
  Pi. Works after the **sysid=2** fix (sysid=1 collided with FC, collapsing
  the route entry).
- **Secondary — UDP intercept:** dashboard → UDP 14551 (LAN-direct or
  Tailscale) → `mavlink_bridge._gcs_to_fc_loop` parses + intercepts locally.

## 5. Deployment / ops

Pi pipeline runs as **systemd units**, auto-start on boot, in `~/Drone/ops/`:

- `skyresq-core.target` pulls the 8 core services.
- `skyresq-sar-orchestrator.service` is opt-in (manual `systemctl start`).
- `install.sh` (one-time, sudo) installs units + enables boot start.
- `status.sh` / `logs.sh` / `start.sh` / `stop.sh` convenience wrappers.
- `config` holds tunables (GCS IP, gimbal host, gains, stream dims).

Cold-boot verified: Pi power-on → 8 services up, zero errors, ~30 s.

## 6. Test status

- **SAR state machine:** `test/test_sar_orchestrator.py` — 14/14 end-to-end
  checks pass (mocked inputs, isolated `ROS_DOMAIN_ID`, no GPS/FC needed).
- **Drop geometry:** exact — 4.00 m from survivor, drone-side.
- **Services:** 8/8 active on cold boot, no restart loops.
- **Dashboard:** `tsc --noEmit` clean. (`next build` needs Node ≥20.9; dev
  uses `npm run dev` on Node 18 — note for CI.)
- **Not yet tested:** real flight; full loop indoors blocked by ArduPilot
  "GUIDED requires position" (no indoor GPS). Next: SITL or outdoor props-off.

## 7. Known issues / deferred (see `AUDIT_2026_05_14.md`)

- **Drop altitude:** orchestrator drops at search altitude (NaN = hold
  current). Needs a DESCEND state + `drop_altitude_m` before real flight.
- **Detector confidence 0.30** is low (false positives); tune to ~0.45 at site.
- **Gimbal Z-1 Mini** locks its single TCP slot 30–90 s after disconnect;
  `RestartSec=45` accommodates this.
- **SECURITY (accepted by owner):** `935d99a` committed `.bench/` SSH helper
  scripts containing the Pi password to **public** history. `.bench/` is now
  gitignored (no new commits include it) but the old commit is permanent.
  Password not rotated per owner decision; exposure limited to Tailscale-only
  reachability. **Do not re-add `.bench/` to git.**

## 8. Pre-demo checklist

See `AUDIT_2026_05_14.md` §"Pre-demo checklist": props off for bench, Pi
static DHCP lease on Blaze, NTP, ArduPilot params (`SERIALn_PROTOCOL=2`,
`FS_GCS_ENABLE=1`, `FENCE_ENABLE=1`, TX mode-switch via `RCx_OPTION`),
dashboard "Pi LAN host" set, SITL run before flight.
