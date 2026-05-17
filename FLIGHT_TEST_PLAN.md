# SAR Drone — First Flight-Test Plan

Phased flight-bring-up plan derived from bench validation (2026-05-16).
Each phase is a gate: **do not proceed until the prior phase passes.**

> Standing rules apply to **every** phase — see bottom.

---

## Phase 0 — Pre-flight bench prep (no flying)

Blockers that MUST be closed before any autonomous geolocation is
trustworthy. Items 1–3 are **coupled — fix as one change**, not piecemeal.

1. **Camera calibration.** `ros2_ws/src/drone_vision/config/camera_intrinsics_z1mini.yaml`
   is a placeholder (`hfov_rad: 1.20 # CONFIRM`). Run a checkerboard
   calibration of the Z-1 Mini at the actual stream resolution and write
   real `fx/fy/cx/cy`. Geolocation is meaningless until this is done.
2. **Resolution scale consistency.** Confirm the detector's real output
   resolution; ensure `geo_localiser` `fx`/`cx` and `image_width/height`
   are all expressed in that same frame (audit found a 320 vs YAML-width
   mismatch — `frames.py` ray angles scale wrong otherwise).
3. **Point the running node at the real intrinsics.** The deployed
   geo_localiser is started by `ops/systemd/skyresq-geo-localiser.service`
   with **no `--ros-args` overrides**, so it always loads the sim
   defaults (`camera_intrinsics_sim.yaml`, 320x240) — verified in the
   live startup banner. The `hardware.launch.py` edit does NOT affect the
   service. Fix is in the **unit ExecStart** (mirror skyresq-mavlink-bridge:
   `--ros-args -p intrinsics_file:=... -p image_width:=... -p image_height:=...`),
   set together with the calibrated values from #1/#2.
4. **Gimbal pitch-sign check.** With manual gimbal control deployed,
   command pitch down from the dashboard and confirm `/gimbal/state`
   reads ≈ **−90°** at nadir (`frames.py` expects negative = down). If it
   reports +90, invert in `gimbal_controller`.
5. **Attitude fusion.** ✅ DONE (commit 42416fe, deployed & running):
   `frames.py` now uses the full Rz·Ry·Rx DCM; geo_localiser feeds
   roll/pitch from `/vehicle/attitude`. Level flight is regression-
   identical to the old yaw-only path.

## Phase 1 — Params & failsafes (bench, props off, RC on)

Verify in Mission Planner and record current values:

- `FENCE_ENABLE=1`, `FENCE_RADIUS` ≤ test-area limit, `FENCE_ALT_MAX`,
  `FENCE_ACTION` (RTL)
- `FS_THR_ENABLE` / radio failsafe → RTL or LAND (this fired on the
  bench when the TX was off — confirm it is deliberate)
- `FS_GCS_ENABLE` and its action (GCS-link-loss behavior)
- `BATT_LOW_VOLT` ≈ 21.6, `BATT_CRIT_VOLT` ≈ 20.4, `BATT_FS_LOW_ACT` /
  `BATT_FS_CRT_ACT` for the 6S pack (matches dashboard voltage alerts)
- `RTL_ALT`, `LAND_SPEED`
- Arming checks **ON** — never disabled to "make it work"
- Rehearse RC override: confirm the TX kill / mode-override disarms or
  takes manual control instantly

## Phase 2 — First hover (props ON, open field)

- Genuine open sky, area clear of people, safety pilot on the sticks.
- Acquire GPS cold-start in the open first (won't lock under cover;
  holds once acquired — verified 2026-05-16).
- Manual STABILIZE/LOITER hover ~2 m for ~20 s. Confirm: stable,
  instant RC override, dashboard telemetry/MODE/battery live, voltage
  alerts behave.
- Land, disarm, inspect.

## Phase 3 — Single autonomous leg

- Upload a tiny 2–3 WP mission, short legs, ~5–10 m altitude.
- Pilot arms, switches to AUTO, finger on the override throughout.
- Confirm it flies the legs and RTLs. Abort to LOITER/manual on any
  anomaly.

## Phase 4 — SAR pattern, no payload

- Draw a small polygon well inside the 400 m limit, generate the
  lawnmower, upload, run in AUTO.
- Walk a person into view; verify detection AND that the geolocated
  marker lands near the real position (validates Phase-0 calibration).
  A few metres of error is expected; "extremely wrong" → recheck
  calibration / gimbal-sign / attitude.

## Phase 5 — Full SAR demo with payload

- Full flow: polygon → search → detect → approach (GUIDED) → drop 4 m
  on the drone side of the survivor → RTL.
- Payload = Tailscale-UDP servo toggle (validated working).

---

## Standing safety rules (every phase)

- **RC transmitter ON, bound, pilot ready — always.** ArduCopter radio
  failsafe disarms within ~1–3 s without it (root-caused 2026-05-16).
- Props OFF for anything that is not an intentional flight test.
- One variable per test; written expected outcome and abort criteria
  defined **before** the test.
- Battery: land by the 21.5 V hard stop. Dashboard alerts at
  22 / 21.5 / 21 / 20.5 V (charge prompt + beep at 20.5).
- Keep the Z-1 Mini / GPS clear of the Pi / 4G / USB stack (GPS
  cold-start needs open sky; once locked the fix holds).

## Known-good as of 2026-05-16 (bench, props off)

Telemetry, GPS fix, arm/disarm, flight-mode readout, SAR mission state
card, mission upload (decoder + single-transport fixes), and the
arm → AUTO → GUIDED → LOITER → RTL autonomous command chain are all
validated on the bench. Manual gimbal control is implemented and
pending Pi deploy + bench test.
