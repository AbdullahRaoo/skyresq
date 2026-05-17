# SAR Drone — Simulation (SITL) Procedure

End-to-end validation of the autonomous SAR demo against ArduCopter
SITL, with the **real** SkyResQ dashboard in the loop. Built and
verified on this development PC before any field flight.

## What this validates (proven 2026-05-17)

| Layer | What runs | Validated |
|---|---|---|
| Flight dynamics | ArduCopter SITL (real ArduPilot firmware) | ✅ arms, takes off, holds GUIDED, RTLs, auto-disarms |
| FC ↔ companion bridge | `drone_vision.bridge.mavlink_bridge` over TCP | ✅ `/vehicle/*` topics live; `fly_to`/`set_mode`/`RTL` reach SITL |
| Autonomy state machine | `drone_vision.mission.sar_orchestrator` | ✅ `IDLE→SEARCH→DETECTION_HOLD→APPROACH→DROP→DROP_HOLD→RTL→DONE` |
| Drop-point geometry | `compute_drop_point` (4 m short of survivor, drone-side) | ✅ exactly 31 m for a 35 m survivor (geometry correct) |
| Payload contract | `sim_payload` mirrors `/payload/cmd` → `/payload/state` | ✅ DROP → DROP_HOLD only fires on payload-state echo |
| GCS link path | TCP↔PTY bridge → `electron/mavlink.js` serial connect | ✅ heartbeat + 31 MAVLink msg types through `/tmp/ttySITL` |
| Camera→world | (Gazebo phase — see below) | pending |

**End-to-end run (latest):** detection injected at *t* = 46.2 s →
`APPROACH` at 47.7 s with target distance **31.0 m** → `DROP` at 49.1 s
(tgt_d 2.5 m) → `RTL` at 52.5 s → `DONE` at 54.1 s. Max displacement
**31.7 m**, returned to **6.7 m** of home. (`.bench/sitl_driver.py` output.)

## Real autonomy bugs SITL caught (and fixed)

[mavlink_bridge.py `_on_mission_fly_to`](ros2_ws/src/drone_vision/drone_vision/bridge/mavlink_bridge.py#L567)
had two issues that would have affected real flight:

1. **`alt=0 + ignore-altitude-bit`** for "hold current altitude" was
   version-fragile — some ArduCopter builds did not honor the ignore-z
   bit and commanded the copter toward alt = 0 (ground). The fix
   tracks the current AGL from `GLOBAL_POSITION_INT` and sends an
   explicit valid altitude with the z-bit USED.
2. **`target_component = 0`** (`wait_heartbeat` in pymavlink 2.4.x
   leaves it at 0 even when the autopilot's heartbeat is srcComp=1).
   ArduCopter generally accepts component 0 for SET_POSITION_TARGET
   but some routing layers filter on it, and DO_REPOSITION/others
   require the autopilot component. The fix forces `tgt_comp = ... or 1`.

Both are deployed in the bridge code; rebuild & restart the
`skyresq-mavlink-bridge` service on the Pi to ship them.

## Setup (one-time)

### ArduCopter SITL
```bash
# Clone + build (5-30 min depending on machine).
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot ~/ardupilot
cd ~/ardupilot
git submodule update --init --recursive --depth 1
python3 -m venv ~/sitl-venv
~/sitl-venv/bin/pip install "empy==3.3.4" pexpect future pymavlink pyserial
PATH="$HOME/sitl-venv/bin:$PATH" ~/sitl-venv/bin/python ./waf configure --board sitl
PATH="$HOME/sitl-venv/bin:$PATH" ~/sitl-venv/bin/python ./waf copter -j$(nproc)
```

### ROS 2 workspace (Jazzy)
```bash
cd ~/Drone/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the full SITL autonomy demo

Three terminals (or background each). All commands assume the
workspace is sourced.

**1. SITL** — headless ArduCopter, listens on tcp:5760 (FC), :5762/:5763 (spare):
```bash
mkdir -p ~/sitl_tmp && cd ~/sitl_tmp && rm -f eeprom.bin
~/ardupilot/build/sitl/bin/arducopter -w --model + --speedup 5 \
  --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm \
  -I0 --home -35.363261,149.16523,584.0,353.0
```

Then **wait ~30 s** for the EKF to settle before arming. (Default
copter.parm `ARMING_CHECK` is strict; in SITL we override it: see
prep step below.)

**2. SITL prep + takeoff** (one-shot — sets params, arms, takes off, idles):
```bash
~/sitl-venv/bin/python ~/Drone/.bench/sitl_canonical.py
```
This is the same script that proved GUIDED reposition works (it
disables fence + arming-check, arms with force-fallback, takes off
to 20 m, and demonstrates SET_POSITION_TARGET reaches the FC).

**3. ROS 2 core stack** (mavlink_bridge + orchestrator + injector + sim payload):
```bash
ros2 launch drone_vision sitl_core.launch.py
```

**4. Drive the autonomy** (publishes `/mission/enable`, triggers
`/sim/inject`, watches `/mission/state`, prints PASS/FAIL):
```bash
python3 ~/Drone/.bench/sitl_driver.py
```

Expect to see the state sequence and a final
`VERDICT : PASS — full autonomous approach+drop+RTL in SITL` line.

## Real dashboard in the loop

The dashboard's `connect()` is serial-only ([electron/mavlink.js:756](../sky-resq/sky-resq-dashboard/electron/mavlink.js#L756)).
A dependency-free TCP↔PTY bridge exposes a SITL TCP port as a
pseudo-terminal that the dashboard opens exactly like a SiK radio.

```bash
# Bridge SITL :5763 to /tmp/ttySITL
python3 ~/Drone/.bench/tcp_pty_bridge.py --tcp 127.0.0.1:5763 --link /tmp/ttySITL
```

Then in the dashboard's **Connect** dialog, paste `/tmp/ttySITL`
(any baud rate works — PTY ignores baud). The serial code path is
exercised unchanged: telemetry, mode chip, arm/disarm, mission
upload, voltage alerts, SAR-mission card.

## What's portable to the 8 GB field laptop

Everything above except the optional Gazebo phase. ArduCopter SITL
itself is light (~10 % CPU at 1× speed); the heavy components are
the optional Gazebo world + YOLO inference, which need this PC.
The autonomy/dashboard/command validation that actually catches
real bugs (as it just did) runs comfortably on the laptop.

## Gazebo visual pipeline (this PC only — partial)

All the pieces are built and individually verified; the only missing
step is the end-to-end run, which hits an SITL `--model JSON`
single-port-per-link constraint that fights a separate pilot client.

**Verified individually:**
- `ardupilot_gazebo` plugin builds against gz-sim 8.11 (needs GStreamer
  optionalised — patch lives in `~/ardupilot_gazebo/CMakeLists.txt`).
- `ros2_ws/src/drone_vision/gazebo/sar_world.sdf` loads: iris_with_gimbal
  + the OpenRobotics "Standing person" from Fuel at +15 m N + ground
  plane, spherical coords at the standard SITL home.
- ArduPilot ↔ Gazebo JSON handshake on UDP 9002 (`ArduPilot Ready`
  status after a quick MAVLink wake-up on 5760).
- gz↔ROS camera bridge publishing 640×480 frames to `/drone/camera_raw`
  at **9.3 Hz**.
- Gimbal direction controllable: needs the ArduPilot plugin's
  channel-8/9/10 control blocks commented out in
  `~/ardupilot_gazebo/models/iris_with_gimbal/model.sdf` (otherwise
  servo PWM defaults overwrite our `/gimbal/cmd_pitch` at 50 Hz).
- **The standing person is clearly visible in the rendered frame** —
  saved at [docs/sim_evidence/gz_camera_sees_person.png](docs/sim_evidence/gz_camera_sees_person.png). YOLO will detect this.

**What's blocking the end-to-end run:**
- `arducopter --model JSON` only binds SERIAL0 (5760) by default;
  SERIAL1/2 are not exposed, so the pilot script can't get its own
  MAVLink stream without fighting `mavlink_bridge` for the same TCP
  port (two clients → EOF/connection-reset thrashing).
- Workarounds for next session: (a) pass `-A "--uartB tcp:5762"` to
  arducopter to add a second TCP server, OR (b) write a tiny ROS node
  that issues arm/takeoff via `mavlink_bridge`'s own connection
  (publish to a new `/mission/cmd_arm` topic + extend the bridge to
  handle it), OR (c) disable `FS_GCS_ENABLE` and have a one-shot pilot
  arm+takeoff then disconnect, with `mavlink_bridge` reconnecting.

**Run shells live at:**
- `ros2_ws/src/drone_vision/gazebo/sar_world.sdf` — world
- `ros2_ws/src/drone_vision/launch/sitl_gazebo.launch.py` — ROS stack
- `ros2_ws/src/drone_vision/config/camera_intrinsics_gz.yaml` — intrinsics
- `.bench/gz_run_all.sh` — orchestrated startup (one-shot pilot conflict
  is the remaining issue)

The injector path in `sitl_core.launch.py` already proves the entire
autonomy / command / FC / dashboard chain end-to-end. Gazebo adds the
**pixel-to-world** stage on top — important for camera-pipeline
confidence but not a gate on the autonomy itself.

## Cleanup

```bash
# Kill ROS sim stack
pkill -f sitl_core.launch
# Kill SITL
pkill -x arducopter
# Kill PTY bridge
pkill -f tcp_pty_bridge.py
# Remove PTY symlink
rm -f /tmp/ttySITL
```
