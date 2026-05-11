# Z-1 Mini Gimbal — Protocol Findings

> Reverse-engineering notes for the XF Robot Z-1 Mini gimbal's TCP control
> protocol on `192.168.144.108:2332`.
>
> **Status (2026-05-11, bench-verified ✓):** protocol implementation
> [gimbal_controller.py](../drone_vision/gimbal/gimbal_controller.py)
> rewritten to match the authoritative ArduPilot driver
> ([`AP_Mount_XFRobot.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mount/AP_Mount_XFRobot.cpp))
> and **physically tested against the real Z-1 Mini**. Both pitch and yaw
> respond to commanded slews; the gimbal also returns its attitude in the
> reply packets.

## What works (RTSP side)

| Path | Value |
|---|---|
| RTSP stream | `rtsp://192.168.144.108:554` (no path) |
| Codec / resolution | H.264 baseline, 1920×1080, 25 fps |
| Server identifies as | `lal0.37.4` (open-source Go RTSP/WebRTC server) |
| Pi consumption | `rtsp://127.0.0.1:8554/skyresq_cam` via mediamtx relay |

A bare `ffprobe` against the gimbal's RTSP port fails with
`Invalid data found when processing input` — only mediamtx's RTSP
client successfully negotiates. Treat the gimbal as "mediamtx-only"
for video; do not point another ffmpeg/cv2 client at port 554.

## Control protocol — corrected

Verified against ArduPilot's
[AP_Mount_XFRobot.h](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mount/AP_Mount_XFRobot.h) +
[`.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mount/AP_Mount_XFRobot.cpp):

### Headers + framing

- **Send header:** `0xA8 0xE5`
- **Recv header:** `0x8A 0x5E`
- **Version:** `0x02` at byte 4 of every frame
- **Length field:** `uint16 LE` at bytes 2-3, contains **total packet size including CRC** (not body length)
- **CRC:** **CRC-16/XMODEM** — poly `0x1021`, init `0x0000`, no reflection, no XOR-out
- **CRC byte order:** **HIGH byte first, then LOW byte** (not little-endian)
- CRC is computed over `bytes[0 : len-2]` (header through final field, but not the CRC bytes themselves)

### Send packet (72 bytes — fixed size, all commands)

The protocol uses **one single 72-byte frame for every command**. Different commands set
different control values and the `order` byte at offset 69. There is no per-order
payload format — the gimbal always reads the full main+sub frame.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0-1 | 2 | header | `0xA8 0xE5` |
| 2-3 | 2 | length | `uint16 LE = 72` |
| 4 | 1 | version | `0x02` |
| 5-6 | 2 | roll_control | `int16 LE` centi-deg, ±18000 |
| 7-8 | 2 | pitch_control | `int16 LE` centi-deg, ±9000 |
| 9-10 | 2 | yaw_control | `int16 LE` centi-deg, ±18000 |
| 11 | 1 | status | Bit0:INS valid, Bit2:control values valid → `0x04` if no AHRS, `0x05` with AHRS |
| 12-17 | 6 | vehicle abs roll/pitch/yaw | int16 LE centi-deg (zero if no AHRS) |
| 18-23 | 6 | vehicle accel N/E/U | int16 LE cm/s² |
| 24-29 | 6 | vehicle vel N/E/U | int16 LE dm/s |
| 30 | 1 | request_code | `0x01` (request sub-frame reply) |
| 31-36 | 6 | reserved | zeros |
| 37 | 1 | sub_header | `0x01` |
| 38-49 | 12 | vehicle lon/lat/alt_amsl | 3× int32 LE (1e7 deg, mm) |
| 50 | 1 | gps_num_sats | uint8 |
| 51-54 | 4 | gps_week_ms | uint32 LE |
| 55-56 | 2 | gps_week | uint16 LE |
| 57-60 | 4 | alt_rel | int32 LE mm above home |
| 61-68 | 8 | reserved2 | zeros |
| 69 | 1 | order | function code, see below |
| 70-71 | 2 | CRC | CRC-16/XMODEM, HIGH byte first |

### Function-order codes

| Code | Name | Use |
|---|---|---|
| `0x00` | NONE | null command (sent to keep prior command's effect) |
| `0x01` | CALIBRATION | |
| `0x03` | NEUTRAL | gimbal returns to centre |
| `0x10` | **ANGLE_CONTROL** | the one we use for SAR — slew to roll/pitch/yaw_control |
| `0x11` | HEAD_LOCK | maintain heading regardless of vehicle yaw |
| `0x12` | HEAD_FOLLOW | match vehicle yaw |
| `0x13` | ORTHOVIEW | |
| `0x14` | EULER_ANGLE_CONTROL | alternative angle command — we initially tried this; **wrong choice** |
| `0x15` | GAZE_GEO_COORDINATES | gaze at a given lat/lon |
| `0x17` | TRACK | track a target locked by CLICK_TO_AIM |
| `0x1A` | CLICK_TO_AIM | snap onto a target at given horizontal/vertical pixel offset |
| `0x20` | SHUTTER | take a still |
| `0x21` | RECORD_VIDEO | start/stop video recording |
| `0x25` | ZOOM_RATE | continuous zoom at rate |
| `0x75` | TARGET_DETECTION | toggle onboard target-detection mode |

For our SAR use, **`0x10` is the only one that matters**. The gimbal's
own onboard detection (`0x75`) could provide an independent lat/lon
estimate we could cross-check against our geo_localiser, but that's
post-demo work.

### Reply packet (72 bytes minimum, same 4-byte prefix)

Bytes 0-3 = header (`0x8A 0x5E`) + length. From byte 4 onward the field
layout differs from send (mode at byte 5, status at 6-7, etc.). The
fields we care about for `/gimbal/state`:

| Offset | Field |
|---|---|
| 18-19 | roll_abs_cd `int16 LE` |
| 20-21 | pitch_abs_cd `int16 LE` |
| 22-23 | yaw_abs_cd `uint16 LE` (0..36000 — wrap to ±180° when republishing) |

## Original mistakes (for future me)

The first implementation got five things wrong:

1. **Order code 0x14 instead of 0x10.** EULER_ANGLE_CONTROL is a different mode the manufacturer added later (and may not be implemented on all firmwares). ANGLE_CONTROL (0x10) is the universal one.
2. **CRC init 0xFFFF (CCITT-FALSE) instead of 0x0000 (XMODEM).** Same polynomial, different starting register.
3. **CRC byte order: low-then-high.** Real protocol is **high-then-low** (network byte order, not little-endian).
4. **Length field meant body-only.** Real protocol = total packet incl CRC.
5. **Packet was ~12 bytes ad-hoc.** Real protocol = fixed 72-byte main+sub frame for every command. The gimbal silently discards anything shorter.

The gimbal silently discards malformed packets (no NAK, no disconnect),
which made every one of these mistakes invisible until we read the
reference driver.

## Bench validation — passed (2026-05-11)

Ran the four-step sequence against the real gimbal:

| Step | Commanded | Observed (visual + log) |
|---|---|---|
| Startup | (defaults yaw=0, pitch=-90) | camera tilted from looking-forward to nadir ✓ |
| 1 | Pan right 30° | gimbal yawed right ✓ |
| 2 | Pan left 60° | gimbal yawed back through centre then left ✓ |
| 3 | Return centre + tilt up 20° | gimbal returned to centre yaw; pitch went -90→-70 (per log) ✓ |
| 4 | Tilt back down to nadir | pitch returned -70→-90 (per log) ✓ |

Reply packets parsed cleanly — `/gimbal/state` reports live roll/pitch/yaw
in degrees on every send tick. Notably:

- Pitch follows commanded angle directly.
- Yaw replies report the **absolute world-frame** orientation
  (`yaw_abs_cd` at bytes 22-23), not the commanded relative yaw. The
  command field (`yaw_control` at bytes 9-10) is relative-to-vehicle,
  but the reply tells us where the camera is actually pointing in the
  world. Both useful: relative for control-loop, absolute for
  geo-localiser.

## Subtle bug found during validation: yaw is signed int16 in the reply

ArduPilot's `AP_Mount_XFRobot.h` declares `yaw_abs_cd` as
`uint16_t (0 ~ +36000)`. **The live firmware in our Z-1 Mini sends signed
int16 centi-degrees** in this field — confirmed on bench: a -30° yaw came
back as 0xF43C, which as uint16 = 62524 (out of the documented range), but
as int16 = -3036 = -30.36°. The parser now reads all three attitude
fields as int16.

## If it still doesn't work — manufacturer app sniff

```bash
# On a laptop on the same 192.168.144.x network as the gimbal:
sudo tcpdump -i any -w gimbal.pcap host 192.168.144.108 and port 2332

# On a phone connected to the gimbal's WiFi: run the vendor app,
# slew clearly (pitch -90→-45→-90, yaw 0→+30→0).

# Inspect:
tshark -r gimbal.pcap -x          # raw bytes
tshark -r gimbal.pcap -V | less    # full decode
```

Things to compare against our packets:

- Header bytes — confirm `0xA8 0xE5`
- Length field — confirm `72` for an angle command
- CRC — re-compute with init=0/poly=0x1021 over `bytes[0:70]`, expect equal to packet bytes 70-71 (high first)
- Order byte at offset 69 — confirm `0x10` for slew commands (or note the alternative)

## Files

- [gimbal_controller.py](../drone_vision/gimbal/gimbal_controller.py) — the protocol implementation
- [bench_gimbal_motion.sh](../../../tools/bench_gimbal_motion.sh) — 4-step motion test (TBD, lives in /tmp currently)
