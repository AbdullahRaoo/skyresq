# SkyResQ Ops

Production-style systemd integration for the SkyResQ SAR pipeline.

## What this replaces

The old workflow ran every ROS node manually via `/tmp/bench_pipeline.sh`,
which got wiped on every reboot. This directory contains:

- One systemd unit per ROS node — independent restart policies, isolated
  failures. If `person_detector` crashes, only it restarts; everything
  else (gimbal, mavlink_bridge, payload_servo) keeps running.
- `skyresq-core.target` — groups the 8 core services so you can
  start/stop the whole pipeline as one unit.
- Auto-start on boot — power the Pi, pipeline comes up within ~30 s.
- Logs go to journald (rotated by the system, queryable with `journalctl`).
- Centralised config in `config` (loaded as systemd EnvironmentFile).

## One-time install

On the Pi:

```bash
cd ~/Drone/ops
chmod +x install.sh start.sh stop.sh status.sh logs.sh
./install.sh
```

The installer asks for sudo, copies units to `/etc/systemd/system/`,
enables auto-start, and brings the pipeline up.

## Day-to-day

| Task                                  | Command                                          |
| ------------------------------------- | ------------------------------------------------ |
| Overview of all services              | `./status.sh`                                    |
| Tail all logs                         | `./logs.sh`                                      |
| Tail one node's log                   | `./logs.sh gimbal-controller`                    |
| Stop everything                       | `./stop.sh`                                      |
| Start everything                      | `./start.sh`                                     |
| Restart one node                      | `sudo systemctl restart skyresq-payload-servo`   |
| Edit pipeline config                  | edit `~/Drone/ops/config`, then `./stop.sh && ./start.sh` |
| Engage SAR autonomy                   | `sudo systemctl start skyresq-sar-orchestrator`  |
| Disengage SAR autonomy                | `sudo systemctl stop skyresq-sar-orchestrator`   |

## What's in the pipeline (core target)

- `skyresq-mavlink-bridge` — Pi↔FC MAVLink, GCS UDP mirror, payload command routing.
- `skyresq-rtsp-camera` — reads gimbal RTSP via mediamtx, publishes `/drone/camera_raw`.
- `skyresq-gimbal-controller` — XF Z-1 Mini TCP control, pixel-error tracking.
- `skyresq-visual-servo` — `/target_position` → `/gimbal/cmd/look_at_pixel` passthrough.
- `skyresq-payload-servo` — drives BCM 16 hobby servo via lgpio (Arduino-equivalent PWM).
- `skyresq-person-detector` — YOLO26-Nano via ncnn, publishes `/detections` and `/target_position`.
- `skyresq-geo-localiser` — fuses detection + gimbal + drone pose → `/target/world` (lat/lon).
- `skyresq-gcs-link` — UDP JSON to dashboard: `survivor_cluster`, `detection_frame`, `pi_status`, `mission_state`.

## Not in the target (opt-in)

- `skyresq-sar-orchestrator` — autonomy state machine (SEARCH → DETECTION_HOLD →
  APPROACH → DROP → RTL). Deliberately off by default; turn on only when the
  drone is staged for autonomous flight. The dashboard's `/mission/enable`
  kill-switch additionally gates real FC actions even when the orchestrator
  is running.

## Updating after a code change

1. Edit code, `pi_put.py` it across.
2. `cd ~/Drone/ros2_ws && colcon build --packages-select drone_vision --merge-install --symlink-install`
3. `sudo systemctl restart skyresq-<node>` (or `./stop.sh && ./start.sh` for all).
