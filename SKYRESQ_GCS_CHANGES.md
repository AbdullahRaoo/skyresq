# SkyResQ GCS — Required Changes

> Spec for changes to [sky-resq-dashboard](https://github.com/AbdullahRaoo/sky-resq-dashboard)
> to support the autonomous SAR drone pipeline + a safe demo mode.
>
> Status: design doc, not yet implemented.

---

## 0. Context

The dashboard today (commit at time of writing) is a single mission-cockpit
view: map + HUD + video iframe + connection controls. Survivor markers exist
in the data model (`survivorStore.ts`) but there is **no UI surface to view
the history, toggle visibility, drop a payload near a survivor, or interact
with detections from the video feed.**

The drone-side pipeline (RPi) will start sending two new things to the
dashboard once the realignment work lands:

1. **Survivor cluster** JSON payloads (one per confirmed detection cluster)
2. **Live detection** JSON payloads (per-frame, with bounding box in image
   coordinates) — so the video can be overlaid with clickable boxes

Both will arrive over the existing UDP/Tailscale channel. SiK fallback
carries only a count + last-known coordinates via `NAMED_VALUE_INT/FLOAT`.

---

## 1. New data flowing in from the drone

### 1.1 Survivor cluster (existing contract — confirmed in ConOps)
```jsonc
{
  "type": "survivor_cluster",
  "id": "cluster-1747043820-3",   // stable across updates
  "count": 3,                     // people in cluster
  "lat": 33.72938,
  "lon": 73.09312,
  "alt": 0.0,                     // ground altitude estimate (m AMSL)
  "confidence": 0.91,             // best detection confidence in cluster
  "first_seen_ms": 1747043820123,
  "last_seen_ms":  1747043829456,
  "n_samples": 18                 // how many detector frames contributed
}
```
Sent on cluster *creation*, *status update*, or every ~2 s while actively
tracked. Idempotent on `id`.

### 1.2 Live detection (NEW — for video overlay)
```jsonc
{
  "type": "detection_frame",
  "frame_ts_ms": 1747043820234,
  "stream_width": 1280,           // matches the WebRTC frame size
  "stream_height": 720,
  "detections": [
    {
      "bbox": [342, 198, 437, 388],   // [x1, y1, x2, y2] in stream pixels
      "confidence": 0.88,
      "class": "person",
      "cluster_id": "cluster-1747043820-3"  // null if unclustered
    }
  ]
}
```
Sent at the **detector's effective rate** (~5–10 Hz). The GCS only needs
the most recent one (drop older if behind). UDP loss is fine — next one
arrives in <200 ms.

### 1.3 Heartbeat from the Pi (existing — keep)
- `STATUSTEXT` over SiK: "RPi: vision OK, 42 dets so far"
- `NAMED_VALUE_INT` on `cluster_count`, `current_state` (mission state),
  `link_4g_ok`. Keeps the dashboard informed even when 4G drops.

---

## 2. New UI surfaces

### 2.1 Video feed — clickable survivor overlay

**File to modify:** [`src/components/video/VideoFeed.tsx`](src/components/video/VideoFeed.tsx)

The current iframe approach can't capture clicks on overlays drawn over
the video. Two paths, in order of preference:

#### Path A (recommended): native `<video>` + `<canvas>` overlay
- Replace the iframe with a `<video>` element pointed at the same
  WebRTC stream (the Pi's mediamtx server should expose a WHEP endpoint —
  if not, that's a small Pi-side change).
- Stack an absolutely positioned `<canvas>` over the `<video>` with
  `pointer-events: none` on the canvas surface, but `pointer-events: auto`
  on rendered marker `<button>` elements.
- A new hook `useDetectionOverlay()` subscribes to detection frames and
  redraws the canvas on each new frame (or uses `requestAnimationFrame` to
  keep it smooth).
- Each detection bbox renders as a green outline. **Cluster centroids** get
  a clickable circular marker badge with the count.
- Clicking a marker calls `useSurvivorStore.setSelected(id)` (existing).

```
┌──────────────────────────────────────────────────┐
│  ●LIVE  WEBRTC                            ⛶      │
├──────────────────────────────────────────────────┤
│  ┌────────────────┐                              │
│  │  ┌────┐        │                              │
│  │  │ ╳3 │ <─── clickable cluster badge          │
│  │  └────┘   bbox outlines                       │
│  │  │   ╳1│                                      │
│  │  └────┘                                       │
│  └────────────────┘                              │
└──────────────────────────────────────────────────┘
```

#### Path B (fallback): keep iframe, overlay outside
If switching off the iframe is too invasive, draw the overlay using
`pointer-events: none` markers positioned on top of the iframe element.
Clicks won't reach the video, but the markers will be clickable. Loses
ability to render bbox outlines accurately if the iframe scales the video
differently (acceptable for clusters; lose bbox precision).

**Acceptance:**
- When a `detection_frame` arrives, outline rectangles appear within
  120 ms over the video at the correct positions, scaled to the player
  size. Cluster badges appear at the centroid of grouped detections.
- Clicking a cluster badge selects it in the survivor store (same effect
  as clicking the corresponding map marker).
- Detections more than 1.5 s old fade out and are removed (no stale
  ghosts on the screen).

---

### 2.2 Survivors page — detection history

**Files to add:**
- `src/app/survivors/page.tsx` — new page
- `src/components/survivors/SurvivorTable.tsx`
- `src/components/survivors/SurvivorFilters.tsx`

**Files to modify:**
- `src/components/layout/Sidebar.tsx` — add a "Survivors" nav entry
- `src/store/survivorStore.ts` — add `visibleIds: Set<string>` + actions
  `setVisibility(id, visible)`, `setAllVisible(visible)`,
  `getVisible()` selector.

**Layout:**

```
┌─ Sidebar ─┬─────────────────────────────────────────────────────┐
│           │  Survivors                                            │
│  Map      │  ┌─────────────────────────────────────────────────┐ │
│  Mission  │  │ Filters: [All] [New] [Confirmed] [Rescued]  ⌕    │ │
│ ►Survivors│  │ Show on map: [✓ All] [✗ None] [Invert]           │ │
│  Settings │  └─────────────────────────────────────────────────┘ │
│           │  ┌───┬─────────┬──────────┬──────┬──────┬──────┬───┐ │
│           │  │ ✓ │ ID      │ Time     │  Lat │  Lon │ Conf │ … │ │
│           │  ├───┼─────────┼──────────┼──────┼──────┼──────┼───┤ │
│           │  │ ✓ │ #003    │ 14:32:08 │ 33.7 │ 73.0 │ 0.91 │ ⤓ │ │
│           │  │ ✗ │ #002    │ 14:31:42 │ 33.7 │ 73.0 │ 0.65 │ ⤓ │ │
│           │  │ ✓ │ #001    │ 14:30:21 │ 33.7 │ 73.0 │ 0.88 │ ⤓ │ │
│           │  └───┴─────────┴──────────┴──────┴──────┴──────┴───┘ │
│           │                                                       │
│           │  Selected: #003 (Confirmed, 3 people)                 │
│           │  [ Highlight on map ]  [ Go to survivor ]  [ Drop ]   │
└───────────┴─────────────────────────────────────────────────────┘
```

**Required columns** (sortable):
- **Show** — checkbox controlling map visibility (per-row override)
- **ID** — short, e.g. last 6 chars of `cluster_id`
- **Time** — local time of first detection
- **Lat / Lon** — copy-on-click
- **People** — count from cluster JSON
- **Confidence** — best confidence
- **Status** — `new` | `confirmed` | `rescued`
- **Actions** — small button column: "Center on map", "Go here",
  "Mark rescued", "Drop payload", "Delete"

**Filters bar:**
- Status pills (All / New / Confirmed / Rescued) with counts
- Free-text search on `id` (or fuzzy on lat/lon for ops who type coords)
- "Show on map": bulk toggle — All / None / Invert

**Per-row actions:**
- **Center on map** — re-routes the user to the map view and pans/zooms
  the map to the survivor's coords with the marker pulsing
- **Go here** — sends the drone toward this lat/lon via a MAVLink
  `SET_POSITION_TARGET_GLOBAL_INT` (already handled by the backend; just
  invoke the existing `/api/goto` endpoint)
- **Mark rescued** — updates `status` to `rescued` (sticks in local
  state; we can persist later)
- **Drop payload** — triggers `MAV_CMD_DO_SET_SERVO` if drone is at this
  survivor (i.e. within auto-drop geofence)
- **Delete** — removes the cluster (false positive cleanup)

**Acceptance:**
- A detected cluster appears as a row in <500 ms of the JSON arriving
- Toggling the checkbox hides/shows the matching map marker immediately
- "Highlight on map" works whether the map is currently shown or not
  (routes to map view if not visible)

---

### 2.3 Map view — visibility controls + layer panel

**File to modify:** `src/app/page.tsx` (or wherever the map lives), plus
a new `src/components/map/MapLegend.tsx`.

The map currently renders all survivors. We need:

- **Per-survivor visibility** driven by `survivorStore.visibleIds`
- **Status filter** — multi-select chips above the map (or in a corner
  legend): show/hide by status
- **Path overlay** (nice-to-have): show the drone's recent track as a
  faint polyline; toggleable
- **Search-area overlay** — already partially there via `missionStore`;
  ensure the demo's small search square renders distinctly from a full
  polygon survey

Legend mock:
```
┌──────────────────────────┐
│ ◉ Drone   • Home   △ WP  │
│ ☑ New (2)  ☑ Confirmed(1)│
│ ☐ Rescued (0)            │
│ ☑ Drone path             │
└──────────────────────────┘
```

---

### 2.4 Payload drop — manual + auto

**Files to modify:**
- `backend/routers/commands.py` — add `/api/payload/drop`,
  `/api/payload/policy`
- `src/store/missionStore.ts` — add `autoDropPolicy` config
- new `src/components/mission/PayloadControl.tsx`

#### Manual drop button
Already implied by the spec but doesn't fully exist yet. Add a prominent
button in the survivor detail panel **and** in a permanent corner of the
mission view. Both call the same `/api/payload/drop` endpoint.

The backend's drop handler:
1. **Validate** interlocks first:
   - Drone is armed in `GUIDED`
   - GPS fix type ≥ 3, HDOP < 2.0
   - AGL within `[min_drop_alt_m, max_drop_alt_m]` (config: 1.5 m to
     10 m by default)
   - Battery remaining > 30 % (refuse to drop on low battery)
   - Has NOT already dropped this mission (one-shot flag, cleared on
     `/api/reset_mission`)
2. **Send** `MAV_CMD_DO_SET_SERVO` with `param1=servo_channel`,
   `param2=pwm_open_us` (e.g. AUX1 → ch 9, 1900 µs open)
3. **Confirm** — listen for the COMMAND_ACK from the FC; respond to the
   frontend with `{ok: bool, reason: str}`
4. **Schedule** a re-close after `open_hold_secs` (default 3 s) using a
   second MAV_CMD_DO_SET_SERVO with `pwm_closed_us` (1100 µs)
5. **Update** the matching survivor's status to `rescued` if a survivor
   was the active context

#### Auto-drop policy
Configurable behaviour that lets the operator say *"if the drone reaches a
survivor and visual confirmation holds, drop without me clicking."* This
makes the demo cleaner and reduces operator workload during the
production path.

Policy object (stored in `missionStore.autoDropPolicy`):
```ts
interface AutoDropPolicy {
  enabled: boolean;             // operator must explicitly turn on
  trigger: "manual" | "auto";   // manual = button only; auto = button + auto
  horizontal_tolerance_m: number;   // default 1.0 — drone must be this close to target
  altitude_min_m: number;           // default 2.0 — never auto-drop below
  altitude_max_m: number;           // default 5.0 — never auto-drop above
  hold_time_s: number;              // default 3.0 — must be in tolerance this long
  require_active_detection: boolean;// default true — detector must still see person
  one_shot: boolean;                // default true — once per mission
}
```

UI surface — a panel in the mission view:
```
┌─ Payload ────────────────────────────┐
│ Status: READY (not dropped)          │
│                                      │
│ ┌──────────────────────────────────┐ │
│ │ [ MANUAL DROP ]   (big red btn)  │ │
│ └──────────────────────────────────┘ │
│                                      │
│ ☐ Auto-drop when within target       │
│   - Tolerance: [ 1.0 ] m             │
│   - Hold:      [ 3.0 ] s             │
│   - Altitude:  [2.0 ─── 5.0 ] m      │
│   - Require live detection: ☑        │
└──────────────────────────────────────┘
```

**Auto-drop state machine (lives in the backend, not the drone):**
```
IDLE  ──policy.enabled & auto──►  ARMED
ARMED ──in_tolerance & detection_fresh──► CONFIRMING
CONFIRMING ──held > hold_time_s──►  DROPPING
CONFIRMING ──drift_out──►  ARMED (reset hold timer)
DROPPING ──MAV_CMD ack──►  DROPPED
DROPPING ──fail──►  ARMED (3 retries then fault)
DROPPED ──one_shot──►  DONE   (no more auto-drops)
```

**Safety rails (non-negotiable):**
- Operator can hit "MANUAL DROP" any time — bypasses auto-drop state
- Operator can hit "DISARM AUTO" any time — sets `enabled=false`
- If `vehicle.mode` leaves `GUIDED`, auto-drop disarms
- If `vehicle.armed=false`, drop is refused (FC's own interlock)
- If link to drone drops > 1 s, auto-drop disarms

**Acceptance:**
- Manual drop button works in both Map and Survivors views
- Auto-drop only fires when ALL configured conditions hold for the
  full `hold_time_s`
- Drop is one-shot per mission; reset requires explicit
  `/api/reset_mission`
- A failed `MAV_CMD_DO_SET_SERVO` (NACK / no ack within 1 s) does NOT
  mark the survivor as rescued and does NOT clear the one-shot flag

---

### 2.5 "Demo Mode" — a constrained, audience-safe workflow

**Files to add:**
- `src/components/demo/DemoModePanel.tsx`
- new `demoStore.ts`

A toggle in the title bar (or a separate route) that puts the dashboard
in a guided walkthrough state:

1. Pre-flight checklist modal (gates Start):
   - [ ] Drone powered on, props attached
   - [ ] GPS fix ≥ 3D (auto-confirmed from telemetry)
   - [ ] Battery > 70 %
   - [ ] Clear sky, no people in flight zone
   - [ ] Spotter assigned
2. Pre-set search area (5 × 5 m square, centred on drone home)
3. Altitude constrained to **3 m max** across all commands
4. Big step-by-step prompt panel:
   - "Click ARM to begin"
   - "Drone is searching… stand by"
   - "Survivor detected! Click marker to investigate"
   - "Visual confirmation — click DROP when ready"
   - "Click RTL to return home"
5. Always-visible HOVER / RTL emergency buttons at top of screen
6. End-of-demo summary card (time, detections, battery used)

In demo mode, the operator's UI is simplified — fewer settings exposed,
larger touch targets, defaults that match the demo profile.

---

## 3. Backend changes summary

| File | Change |
|---|---|
| `backend/routers/commands.py` | Add `/api/payload/drop`, `/api/payload/policy`, `/api/goto`, `/api/reset_mission` |
| `backend/routers/telemetry_ws.py` | Subscribe to a new UDP socket for incoming `detection_frame` + `survivor_cluster` JSON from the Pi; forward to WebSocket clients |
| `backend/services/payload_service.py` (new) | The auto-drop state machine (FSM described above) |
| `backend/models/drone_state.py` | Add `detections: list[DetectionFrame]`, `payload_state: PayloadState` |
| `backend/models/payload.py` (new) | Pydantic models for `AutoDropPolicy`, `PayloadState`, `DropPayloadRequest`/`Response`, `DetectionFrame` |

---

## 4. Frontend store additions

```ts
// src/store/survivorStore.ts — additions
visibleIds: Set<string>;
setVisibility: (id: string, visible: boolean) => void;
setAllVisible: (visible: boolean) => void;
markRescued: (id: string) => void;

// src/store/missionStore.ts — additions
autoDropPolicy: AutoDropPolicy;
setAutoDropPolicy: (p: Partial<AutoDropPolicy>) => void;
payloadState: "ready" | "armed" | "confirming" | "dropping" | "dropped" | "fault";

// src/store/demoStore.ts — new
demoMode: boolean;
currentStep: number;
checklist: Record<string, boolean>;
```

---

## 5. Phased rollout

| Phase | Scope | Dependency |
|---|---|---|
| **Phase 1 — wiring** | Backend endpoint for the new UDP detection socket; WebSocket relay to frontend; survivor cluster updates already work; add visibility filters in Map | Pi side starts sending survivor_cluster JSON |
| **Phase 2 — survivors page** | New `/survivors` route, table, filters, per-row actions, bulk visibility | Phase 1 |
| **Phase 3 — payload service** | Manual drop button + backend service + interlocks; one-shot guard | Phase 1 |
| **Phase 4 — video overlay** | Switch from iframe to `<video>` + `<canvas>`; render bbox + cluster badges; click-to-select | Phase 1 + Pi sends `detection_frame` |
| **Phase 5 — auto-drop policy** | FSM in backend + policy UI; safety rails | Phase 3 |
| **Phase 6 — demo mode** | Guided walkthrough, checklist, constrained envelope, larger emergency buttons | Phases 3 + 4 |

Phases 1–3 are independent and can land in parallel with the Pi-side
realignment work. Phase 4 is the biggest UI change and benefits from
having a working drone-side detection stream first. Phases 5 and 6 build
on the previous phases.

---

## 6. Things explicitly out of scope (for now)

- **Persistent storage** of survivor history across GCS restarts. Right
  now it lives in Zustand. Add SQLite later if needed.
- **Multi-drone support.** All current designs assume one drone.
- **3D / terrain-aware drop**. We use flat-earth + AGL from baro/GPS.
  Good enough for the demo and most SAR sites.
- **Vendor-locked mobile app.** Stay browser/Electron.

---

## 7. Implementation gaps found in code review

> These are stubs or missing handlers identified in the current dashboard
> codebase. They must be filled before the features in §2 work end-to-end.

### 7.1 Electron main process (`main.js`) — three stub IPC handlers

**`mavlink-upload-mission`** (line ~100 in main.js):
```js
// CURRENT — does nothing real
console.log(`[Main] Mission upload requested: ${waypoints.length} waypoints`);
return { success: true, message: `${waypoints.length} waypoints ready` };
```
Needs the full ArduPilot mission upload handshake:
1. Send `MISSION_COUNT` (count = waypoints.length, target = FC)
2. Wait for `MISSION_REQUEST_INT` for each seq 0…N-1
3. Reply with `MISSION_ITEM_INT` (lat/lon in 1e7 ints, alt in m, command from waypoint)
4. Wait for `MISSION_ACK` (type == MAV_MISSION_ACCEPTED)
5. Send `MAV_CMD_DO_SET_MISSION_CURRENT` seq=0 to arm the new mission

**`mavlink-fly-to`** (main.js, after the upload handler):
```js
// CURRENT — logs coords, does nothing
return { success: true, message: `Flying to ${lat.toFixed(6)}, ${lon.toFixed(6)}` };
```
Needs:
1. Set mode to `GUIDED` (`set_mode(4)` for ArduCopter)
2. Send `SET_POSITION_TARGET_GLOBAL_INT`:
   - `type_mask = 0b110111111000` (position-only, ignore vel/accel/yaw)
   - `coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`
   - `lat_int = lat * 1e7`, `lon_int = lon * 1e7`, `alt` in metres AGL

**`mavlink-deploy-payload`**:
```js
// CURRENT — IPC channel registered in preload.js but NO handler in main.js at all
```
Needs:
1. Run interlocks (armed, GUIDED, GPS fix ≥ 3, AGL in [1.5, 10] m, battery > 30 %)
2. Send `MAV_CMD_DO_SET_SERVO` param1=servo_channel, param2=pwm_open_us
3. Wait for COMMAND_ACK
4. Schedule close: send same command with pwm_closed_us after open_hold_secs
5. Return `{success, reason}` to renderer

### 7.2 `GimbalControl.tsx` — UI only, no IPC

The current component updates local state for pitch/yaw but the `applyPreset`
callback has a `// TODO: Send MAV_CMD_DO_MOUNT_CONTROL via IPC` comment and
does nothing. Add:
```ts
// In applyPreset:
if (window.electron) {
    window.electron.setGimbalAngle(p, y);  // new IPC channel needed
}
```
And the corresponding `main.js` handler using `MAV_CMD_DO_MOUNT_CONTROL`:
- param1 = pitch (deg)
- param2 = 0 (roll)
- param3 = yaw (deg)
- param7 = 2 (MAV_MOUNT_MODE_MAVLINK_TARGETING)

### 7.3 `backend/routers/commands.py` — missing `/api/goto`

The survivors page (§2.2) and payload panel (§2.4) both reference a
`/api/goto` endpoint that does not exist in `commands.py`. Add:
```python
class GotoRequest(BaseModel):
    lat: float
    lon: float
    alt: float = 10.0  # metres AGL, capped server-side

@router.post("/goto", response_model=CommandResponse)
async def goto(request: GotoRequest) -> CommandResponse:
    """Switch to GUIDED and fly to lat/lon/alt."""
    ...
```
The handler mirrors the `mavlink-fly-to` IPC logic above but runs in the
Python backend (for browser/web-serial mode). Both paths must exist.

### 7.4 `backend/routers/telemetry_ws.py` — no UDP socket for Pi detections

The backend currently only reads from the serial MAVLink connection. There is
no code to open a UDP socket and receive the `detection_frame` /
`survivor_cluster` JSON packets from the Pi. This is the single biggest
missing piece for Phase 1. See §8.3 for the protocol spec.

### 7.5 `electron/mavlink.js` — required in `main.js` but not in the repo

`main.js` does `require('./electron/mavlink')` and wraps the `MAVLinkHandler`
class. This file is not committed (likely in `.gitignore` or never written).
It must be created alongside the IPC handlers above. Minimum API:
```js
class MAVLinkHandler {
    connect(connectionString, baudRate) → Promise<{success, message}>
    disconnect() → Promise<{success, message}>
    arm() → Promise<{success, message}>
    disarm() → Promise<{success, message}>
    setMode(modeName) → Promise<{success, message}>
    getConnectionProfiles() → Array<ConnectionProfile>
    // New — required by §7.1, §7.2:
    uploadMission(waypoints) → Promise<{success, message}>
    flyTo(lat, lon, alt) → Promise<{success, message}>
    deployPayload() → Promise<{success, message}>
    setGimbalAngle(pitch, yaw) → Promise<{success, message}>
}
```

---

## 8. Recommendations on open design questions

### 8.1 Video overlay — use Path B now, migrate to Path A post-demo

**Recommendation: ship Path B (iframe + external overlay) for the demo.**

The `VideoFeed.tsx` iframe points at `http://<pi-ip>:8889/skyresq_cam`.
Switching to native `<video>` requires the Pi's mediamtx server to expose a
WHEP endpoint. The current Pi config is unknown — adding WHEP may be trivial
or may need a mediamtx upgrade and config change.

For the demo, Path B is faster and lower risk:
- Keep the `<iframe>` as-is
- Add an absolutely-positioned `<div>` layer over it with `pointer-events: none`
- Render cluster-centroid badges (not bbox outlines) on that layer —
  cluster positions are in lat/lon, converted to video coordinates using the
  gimbal's projection math (frames.py already has this)
- Badge clicks work fine; only per-pixel bbox accuracy is lost

Path A (native `<video>` + `<canvas>`) becomes the target once the Pi is
confirmed to serve WHEP. Add this to the mediamtx config on the Pi:
```yaml
# /etc/mediamtx.yml — add under the path entry for skyresq_cam:
paths:
  skyresq_cam:
    source: rtsp://192.168.144.108/...
    webrtcEnabled: true          # already enabled for the iframe
    webrtcICEHostNAT1To1IPs: []
```
WHEP endpoint will then be at:
`http://<pi-ip>:8889/skyresq_cam/whep` — use that as the `<video>` `src`.

### 8.2 Servo channel and PWM values

**Recommendation: default to AUX1, make configurable via `.env`.**

Add to the `.env` (and `backend/config.py`):
```
PAYLOAD_SERVO_CHANNEL=9       # AUX1 = ch 9 on ArduCopter
PAYLOAD_PWM_OPEN_US=1900      # servo open (latch released)
PAYLOAD_PWM_CLOSED_US=1100    # servo closed (latch held)
PAYLOAD_OPEN_HOLD_S=3         # seconds to hold open
```
In `config.py`:
```python
payload_servo_channel: int = 9
payload_pwm_open_us: int = 1900
payload_pwm_closed_us: int = 1100
payload_open_hold_s: float = 3.0
```
**Before first flight**: confirm the channel in Mission Planner under
`SERVO9_FUNCTION` (should be set to `RCPassThru` or a specific servo type)
and test the full open/close cycle on the bench with props off.

### 8.3 Pi → GCS UDP protocol

The Pi's `gcs_link` ROS node will send JSON packets to the GCS over UDP.
Agreed protocol:

| Parameter | Value |
|---|---|
| Pi sends to | `GCS_TAILSCALE_IP:5005` (configured in Pi's `.env`) |
| GCS listens on | `0.0.0.0:5005` UDP |
| Format | newline-delimited JSON (one JSON object per datagram) |
| Max datagram size | 64 KB (well within UDP limit; `detection_frame` with 20 boxes ≈ 1 KB) |
| Loss handling | UDP — loss is OK; stale `detection_frame` discarded by timestamp |
| Auth | None for now (Tailscale provides network-layer auth) |

Add to GCS `.env`:
```
PI_DETECTION_HOST=0.0.0.0
PI_DETECTION_PORT=5005
GCS_TAILSCALE_IP=100.123.87.26   # Pi's Tailscale IP (already in VideoFeed.tsx)
```

The GCS backend opens this socket in `telemetry_ws.py` lifespan and
dispatches messages by `"type"` field:
- `"survivor_cluster"` → `survivorStore` update + WebSocket broadcast
- `"detection_frame"` → most-recent-only buffer + WebSocket broadcast
- anything else → logged and dropped

### 8.4 Demo mode — one codebase, one toggle

**Recommendation: single codebase, `DEMO_MODE` toggle in the title bar.**

Two separate codebases would diverge immediately and double maintenance
burden. The demo-mode overlay approach from §2.5 is the right call:

- `demoStore.ts` holds `demoMode: boolean` (persisted to `localStorage` so
  a page refresh during the demo doesn't reset it)
- A small "DEMO" badge in the title bar; clicking it toggles the mode
  (requires clicking through a confirmation — can't accidentally enter demo
  mode during a real mission)
- Demo mode gates: 3 m altitude cap enforced server-side (the `/api/goto`
  endpoint and the mission upload handler both clamp `alt` to 3.0 when
  `settings.demo_mode` is `True`), pre-set 5×5 m search area, step-by-step
  prompts panel, and constrained UI (fewer settings exposed)
- `DEMO_MODE=true` can also be set in `.env` to hard-lock the GCS into demo
  mode for an event (so it can't be accidentally toggled off)

Add to `backend/config.py`:
```python
demo_mode: bool = False
demo_max_alt_m: float = 3.0
demo_search_radius_m: float = 5.0
```
