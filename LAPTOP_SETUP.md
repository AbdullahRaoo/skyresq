# Field Laptop Setup — SkyResQ Operator Workstation

Target: **i7-5600U / 8 GB / Ubuntu 24.04**. Role: run the SkyResQ
dashboard against the **real drone** in the field over SiK radio +
Tailscale. **No SITL, no Gazebo, no ROS** on the laptop — the ROS
stack runs on the Pi; the dashboard is a pure GCS (Node/Electron +
serialport). Simulation stays on the dev PC.

> Validation status before this move: autonomous flight + approach +
> payload drop are SITL-proven and the camera→YOLO→geo pixel-to-drop
> chain is Gazebo-proven. See `SIMULATION.md`. The laptop just flies
> the *real* bird with the validated software.

---

## 0. What moves vs what stays

**Bring (git):** `Drone/` and `sky-resq-dashboard/` repos — already
pushed; just clone on the laptop.

**Bring (NOT in git — transfer manually, see §4):**
- `sky-resq-dashboard/.env` — field config; `.env.example` is NOT a
  substitute (real values: Pi Tailscale IP, `MAVLINK_UDP_BIDIR=1`).
- The Claude memory dir (this chat's context): the dev PC's
  `~/.claude/projects/-home-abdullah-Drone/memory/` (13 files).

**Leave on the dev PC (do NOT copy):** `~/ardupilot`, `~/ardupilot_gazebo`,
`~/sitl-venv`, `~/sitl_tmp`, `Drone/.bench/`. The laptop never runs sim.

---

## 1. System prerequisites (laptop, one-time)

```bash
# Node 20+ (dashboard package.json engines requires >=20.9.0;
# Ubuntu 24.04 ships 18). Use nvm to avoid touching system node.
curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
exec $SHELL
nvm install 20 && nvm use 20 && nvm alias default 20

# Build tools for the serialport native module rebuild
sudo apt update
sudo apt install -y build-essential python3 git

# Tailscale (to reach the Pi's UDP MAVLink mirror + SAR JSON)
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up        # log in with the SAME tailnet as the Pi

# Serial access for the SiK radio (USB) — no sudo to use the port
sudo usermod -aG dialout $USER
# log out / back in (or `newgrp dialout`) for this to take effect
```

## 2. Clone the repos

Clone `Drone/` to the **same path** the dev PC used so the Claude
memory project key matches (`-home-<user>-Drone`). If the laptop
username is also `abdullah`, `~/Drone` works as-is; otherwise adjust
and see §4 note.

```bash
mkdir -p ~/skyresq && cd ~/skyresq      # or wherever you like
git clone https://github.com/AbdullahRaoo/skyresq.git Drone
git clone https://github.com/AbdullahRaoo/sky-resq-dashboard.git
```

## 3. Dashboard install

```bash
cd ~/skyresq/sky-resq-dashboard
nvm use 20
npm install            # builds serialport native bindings (needs §1 tools)
# if serialport fails to bind later: npm run rebuild
```

## 4. Restore the non-git pieces

**a) Field `.env`** — copy the dev PC's
`sky-resq-dashboard/.env` to the same path on the laptop (USB / scp /
Tailscale). Current field values are in the migration bundle
(`SkyResQ_laptop_bundle.tar.gz` — see §6) as `dashboard.env`.

**b) Chat / project context for Claude Code on the laptop** — the
memory files are in the bundle under `memory/`. On the laptop:

```bash
# Path mirrors the clone location: /home/<user>/Drone -> -home-<user>-Drone
mkdir -p ~/.claude/projects/-home-$USER-Drone
cp -r memory ~/.claude/projects/-home-$USER-Drone/
```

Then `claude` started from `~/skyresq/Drone` on the laptop loads
`MEMORY.md` + all notes — same project context as this session
(SITL/Gazebo validation, geo-audit, RC/arming, payload path, etc.).

## 5. Field bring-up checklist (every session)

1. `tailscale status` — confirm the Pi (`100.123.87.26`) is reachable
   (`ping 100.123.87.26`).
2. Plug the SiK radio; confirm it enumerates: `ls /dev/ttyUSB*`
   (usually `/dev/ttyUSB0`). If not ttyUSB0, set
   `MAVLINK_CONNECTION_STRING` in `.env`.
3. `cd sky-resq-dashboard && nvm use 20 && npm run dev`.
4. In the dashboard Connect dialog use the SiK serial port (default
   `/dev/ttyUSB0` @ 57600). Telemetry also fails over to the Pi
   Tailscale UDP mirror (`100.123.87.26`, port 14550;
   `MAVLINK_UDP_BIDIR=1` means commands can also go out UDP:14551).
5. Sanity: telemetry live, MODE/arm chip correct, SAR mission card
   populates, battery voltage alerts behave. (All SITL-validated;
   confirm against the real bird.)

Then resume the phased plan in `FLIGHT_TEST_PLAN.md` (Phase 0
hardware-gated items first: camera calibration, geo-localiser systemd
intrinsics, gimbal pitch-sign, failsafe/geofence audit).

## 6. Migration bundle

The dev PC produced `~/SkyResQ_laptop_bundle.tar.gz` containing:
- `dashboard.env` — the field `.env`
- `memory/` — the 13 Claude memory files (chat context)
- `BUNDLE_README.txt` — restore steps (mirrors §4)

Move it to the laptop by whatever's handy (USB stick, or over
Tailscale once both machines are on the tailnet:
`scp dev-pc:~/SkyResQ_laptop_bundle.tar.gz .`).

## Performance note (8 GB laptop)

`npm run dev` runs the Next dev server + Electron — fine on 8 GB if
you're not also running heavy apps. If it's sluggish in the field,
`npm run build` once on mains power, then `npm start` (lighter than
the dev server). Never run sim on the laptop — that's the dev PC's job.
