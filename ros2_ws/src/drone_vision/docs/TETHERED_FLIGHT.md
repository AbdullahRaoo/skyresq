# Tethered Flight Runbook

> First-flight procedure for the SkyResQ drone. The goal of a tethered
> flight is to validate the full pipeline end-to-end **while the drone
> is physically restrained** so any software fault is non-catastrophic.
>
> Audience: pilot + spotter. Do **not** fly without a spotter.

---

## 0. Required equipment

- [ ] Drone with props attached, fresh charged 4S battery
- [ ] Cube Black powered, **safety switch pressed**, props attached and torqued
- [ ] Raspberry Pi 4 powered (separate BEC or USB-PD source — NOT shared with FC servo rail)
- [ ] Z-1 Mini gimbal powered, network reachable
- [ ] RC TX with mode switch reachable (STABILIZE / LOITER / LAND on three positions)
- [ ] Ground station laptop running SkyResQ + Mission Planner
- [ ] **Physical tether** — 3 m of climbing rope or webbing, attached to a 4-point harness on the drone frame and anchored to ground stake or 25 kg sandbag
- [ ] Fire extinguisher within reach (LiPo precaution)
- [ ] First-aid kit
- [ ] Spotter

---

## 1. Bench checks (motors-off, in the workshop)

### 1.1 Pi → FC link

```bash
ssh pi@<pi-ip>
cd ~/Drone/ros2_ws/src/drone_vision
python3 tools/preflight_check.py
```

Expected: all checks pass, GPS may be `WARN` (indoor). RC link can be `FAIL`
if TX is off — that's fine for the bench check.

### 1.2 ROS 2 pipeline

```bash
# Start the pipeline manually (NOT via systemd yet)
source /opt/ros/jazzy/setup.bash
source ~/Drone/ros2_ws/install/setup.bash
ros2 launch drone_vision hardware.launch.py gimbal_backend:=tcp
```

In a second SSH session, verify each topic produces data:

```bash
ros2 topic hz /vehicle/attitude       # ~10 Hz
ros2 topic hz /drone/camera_raw       # 15–30 Hz depending on stream
ros2 topic hz /gimbal/state           # 20 Hz
ros2 topic echo /target_position --once  # only when a person is in frame
```

If any topic is silent: kill and check that node's logs (`ros2 node info <node>`,
`journalctl -u drone-ros2`). Do NOT proceed if `mavlink_bridge` cannot reach
the FC or the camera stream is dead.

### 1.3 Payload servo bench test (drone disarmed, props OFF)

From Mission Planner, with the drone disarmed:

```
Actions → Servo → Channel: 9 → PWM: 1900 µs → Run
```

The latch should open. Wait 3 s and re-issue with `PWM: 1100 µs` — latch closes.

Verify your dummy package falls cleanly when the latch opens. Reset the
mechanism for flight.

---

## 2. Outdoor pre-flight (drone tethered, motors still off)

1. Stake or sandbag the tether anchor — **drone cannot drift more than 3 m
   in any direction**.
2. Power on in order: FC → Pi → gimbal → RC TX (last).
3. Wait 60 s after FC power-on for GPS to acquire fix and IMU to settle.
4. Run preflight again, outdoors:

   ```bash
   python3 tools/preflight_check.py
   ```

   All checks must be green. **Do not proceed on any FAIL.**

5. On the GCS laptop:
   - Connect SkyResQ → confirm telemetry HUD shows live attitude, GPS fix,
     battery, mode = `STABILIZE`.
   - Confirm survivor map shows the drone marker at the correct location.
   - Confirm video feed renders the camera view (Tailscale link).

6. Pilot dry-run on TX (motors still disarmed):
   - Move sticks — verify each input mirrors in the SkyResQ attitude
     indicator (roll/pitch/yaw response).
   - Cycle mode switch through STABILIZE / LOITER / LAND — confirm flight
     mode updates on GCS within 200 ms.

---

## 3. First arm + hover (tethered, 1 m altitude)

> **Spotter watches the drone. Pilot watches the GCS.** If anything is
> ambiguous, the pilot calls "ABORT" and the spotter physically holds
> the tether to prevent tip-over.

1. Final spotter call: "Clear above, clear below, clear lateral."
2. Pilot calls "Arming" and arms via TX in STABILIZE.
3. Slow throttle increase to ~30 %; the drone should lift to the tether's
   slack length (~1 m).
4. **Hover for 30 s.** Watch for:
   - Roll/pitch drift > 5° → land, recalibrate
   - Throttle saturation (drone won't lift) → land, check battery
   - Unexpected mode change → land, check failsafes
   - SkyResQ link drops > 2 s → land, check Tailscale
5. Pilot calls "Descending" and slowly cuts throttle. Touchdown should be
   gentle — the tether absorbs any tip.
6. Disarm via TX.

**Stop here on the first day.** Power down, debrief, log any anomalies.

---

## 4. Subsequent flights — GUIDED mode + detection

Only on day 2+ after a clean tethered hover:

1. Place the dummy survivor (mannequin or jacket+shoes) ~3 m from the
   take-off point.
2. Repeat sections 2 and 3 (preflight + tethered hover).
3. On the GCS, switch flight mode to `GUIDED` from SkyResQ.
4. Click the dummy's lat/lon on the map → "Fly here".
5. **Pilot keeps hands on TX with mode switch ready** to revert to
   STABILIZE if the drone misbehaves.
6. Confirm:
   - Drone tracks toward the dummy at low velocity
   - Detector publishes `/target_position` (visible in `ros2 topic echo`)
   - Survivor marker appears on the SkyResQ map within a few seconds
   - Gimbal tracks the dummy (visible in the live video)
7. Pilot calls "Hover" — GCS commands LOITER.
8. **Manual** drop via GCS → confirm payload falls onto/near the dummy.
9. GCS commands RTL → drone returns to launch, lands, disarms.
10. Recover the payload, reset the latch, debrief.

---

## 5. Abort criteria — pilot will land immediately if:

- Attitude indicator shows roll or pitch > 30° unexpectedly
- Battery < 30 %
- GPS fix lost (HDOP > 3 or fix_type < 3)
- Mode change occurred that pilot did not command
- Drone moves opposite to commanded direction (compass error)
- Wind gust visibly pushes drone outside its tether length
- Anyone yells "ABORT"

**Abort action: pilot switches TX mode to `LAND`. Drone descends straight
down. Spotter holds tether to prevent tip.**

---

## 6. Post-flight

- [ ] Inspect props for chips or cracks
- [ ] Inspect tether attachment points for stress marks
- [ ] Download the `journalctl -u drone-ros2` log from the Pi
- [ ] Save the Mission Planner DataFlash log (`logs/` directory)
- [ ] Save the SkyResQ session log
- [ ] Note battery final voltage and consumed mAh
- [ ] Brief team — what worked, what didn't, what to fix next session
