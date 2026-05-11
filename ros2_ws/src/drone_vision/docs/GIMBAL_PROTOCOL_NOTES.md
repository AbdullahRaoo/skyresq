# Z-1 Mini Gimbal — Protocol Findings

> Bench-test results from connecting to the XF Robot Z-1 Mini gimbal
> over its proprietary TCP control protocol on `192.168.144.108:2332`.
>
> **TL;DR:** TCP connection succeeds, but our current
> `EULER_ANGLE_CONTROL (0x14)` packets are accepted by the socket and
> then silently discarded by the gimbal — no movement, no state
> feedback. The protocol implementation in
> [gimbal_controller.py](../drone_vision/gimbal/gimbal_controller.py)
> needs the correct payload byte layout before it will work.

## What works (RTSP side)

| Path | Value |
|---|---|
| RTSP stream | `rtsp://192.168.144.108:554` (no path, mediamtx pulls this directly) |
| Codec / resolution | H.264 baseline, 1920×1080, 25 fps |
| Server identifies as | `lal0.37.4` (an open-source Go RTSP/WebRTC server) |
| Pi consumption | `rtsp://127.0.0.1:8554/skyresq_cam` via mediamtx local relay |

A bare `ffprobe` against the gimbal's RTSP port **fails** with
`Invalid data found when processing input` — only mediamtx's RTSP
client successfully negotiates with the server. Treat the gimbal as
"mediamtx-only" for video; never point another ffmpeg/cv2 client
straight at port 554.

## What we tried for control (and what happened)

### Connection layer — works

- TCP connect to `192.168.144.108:2332` succeeds (sub-second handshake)
- Connection stays up indefinitely while we send packets at 20–50 Hz
- The gimbal does **not** disconnect or NAK invalid packets — it just
  silently drops them, which made debugging much harder

### Protocol layer — wrong

Our [gimbal_controller.py](../drone_vision/gimbal/gimbal_controller.py)
encodes packets as:

```
out: [0xA8, 0xE5, 0x02, len_lo, len_hi, order, ...payload..., crc_lo, crc_hi]
in:  [0x8A, 0x5E, 0x02, len_lo, len_hi, order, ...payload..., crc_lo, crc_hi]
```

with CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Order codes guessed
from AP_Mount_XFRobot.cpp:

| Code | Name |
|---|---|
| 0x14 | EULER_ANGLE_CONTROL (pitch, yaw, roll — int16 centi-deg, LE) |
| 0x10 | ANGLE |
| 0x12 | HEAD_FOLLOW |
| 0x17 | TRACK |
| 0x1A | CLICK_TO_AIM |
| 0x20 | SHUTTER |
| 0x21 | RECORD |
| 0x25 | ZOOM_RATE |
| 0x75 | TARGET_DETECTION_TOGGLE |

**Bench result (2026-05-11):** sent a 4-step motion sequence
(pan +30°, pan -60°, return-centre + tilt +20°, tilt -20°). Gimbal
did not move at all. No `/gimbal/state` packets received in the inbound
direction either — confirming the gimbal isn't acknowledging anything.

## What we know is wrong

Some combination of:

1. **Wrong header / version byte.** Maybe `0xA8 0xE5` isn't the request
   header, or the version byte isn't `0x02`.
2. **Wrong length-field semantics.** Our length includes the order
   byte; real protocol might exclude it, or include header + version.
3. **Wrong payload layout for 0x14.** Sign convention, byte order,
   field order, or units (centi-deg vs deci-deg vs raw) could all be off.
4. **CRC over a different range.** Our CRC covers `version+len+order+payload`.
   Real protocol might cover header→payload or use a different poly.
5. **0x14 isn't the right order code** for our model/firmware version.

## Recommended next steps (in priority order)

### 1. Capture traffic from the manufacturer's app (fastest path to truth)

The Z-1 Mini ships with an Android app. The app necessarily speaks the
correct protocol. Capture a session:

```
# On a laptop on the same network as the gimbal:
sudo tcpdump -i any -w gimbal.pcap host 192.168.144.108 and port 2332

# Then on the phone, run the app, point at the gimbal, and slew it
# clearly: pitch -90 → -45 → -90 again, yaw 0 → +30 → 0.

# Stop tcpdump and inspect:
tshark -r gimbal.pcap -V | less
```

The slew commands will reveal:
- Real outbound header
- Real length encoding
- Real EULER payload layout (look for centi-deg values near ±9000, ±4500, ±3000)
- Real CRC scheme (XOR our re-computed CRC against the captured one)

### 2. Re-read the ArduPilot driver source

Our protocol notes came from a third-party summary of `AP_Mount_XFRobot.cpp`.
Reading the **actual** C++ source directly — particularly the packet
construction and CRC functions — will resolve any second-hand
inaccuracies. Path in the ArduPilot tree:

```
libraries/AP_Mount/AP_Mount_XFRobot.cpp
libraries/AP_Mount/AP_Mount_XFRobot.h
```

### 3. Workaround for first flight — lock at nadir via vendor app

If the protocol fix isn't ready, use the Z-1 Mini's own app to
manually position the gimbal at pitch=-90° (nadir) and lock it.
Configure `gimbal_controller backend:=sim` on the Pi (or simply don't
launch the node) and treat the camera as fixed in geo_localiser. The
fixed-camera math still works for our demo — we just lose
target-tracking gimbal slewing.

## Tested-but-broken artefacts kept in the repo

[gimbal_controller.py](../drone_vision/gimbal/gimbal_controller.py)
already contains:

- Working TCP reconnect/timeout logic
- Working `socket.timeout` handling (commit `4de8331`)
- Correctly-applying pixel-error → angle setpoint math
- Working `backend: sim` echo for bench-testing the rest of the pipeline
- Working CRC-16/CCITT-FALSE implementation that future fixes can keep

Once the correct payload layout is known, the only code that needs to
change is `build_euler_angle_packet()` and the inbound `parse_state_packet()`
payload extraction. The framing scaffolding is sound.

## Reference: tools we'll use when fixing this

```bash
# Sniff while exercising the app
sudo tcpdump -i wlan0 -w gimbal_sniff.pcap host 192.168.144.108 and port 2332

# View raw bytes
xxd gimbal_sniff.pcap
tshark -r gimbal_sniff.pcap -x

# Test a single hand-crafted packet via netcat (replace BYTES with hex pairs)
echo -n -e "\xA8\xE5\x02..." | nc 192.168.144.108 2332
```
