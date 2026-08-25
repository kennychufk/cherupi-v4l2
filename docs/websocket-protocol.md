# cherupi-v4l2 WebSocket Protocol

This document specifies the wire protocol between a client and the `camera_ws_server`. It covers connection and client roles, control commands (JSON text), server responses (JSON text), and the binary chunked frame format.

## 1. Connection

| Item | Value |
|---|---|
| URL | `ws://<host>:9001/` for a **commander**, `ws://<host>:9001/observer` for an **observer** (§1.1) |
| Subprotocol | none |
| Compression | disabled |
| Max payload | 16 MiB |
| Idle timeout | 120 s (the server pings idle sockets, so a silent observer stays connected as long as its library answers pings — browsers and the usual client libraries do) |
| Max backpressure | 10 MiB |
| Concurrent clients | **1 commander + 1 observer** — a further connection in a role whose slot is taken gets HTTP `503 Service Unavailable` during upgrade, with body `Server already has a <role> client` |

On client disconnect the server keeps cameras running (so frame processing can continue, and so the other client keeps receiving frames) and only releases that role's slot.

### 1.1 Client roles

A connection holds one of two roles, fixed at upgrade time by the URL path:

| Role | Path | Access |
|---|---|---|
| **commander** | `/` (also `/commander`, or any path other than `/observer`) | Read-write. Owns the camera lifecycle (`configure` … `stop_cameras`), the camera attributes (`set_lens_position`, `set_exposure_time`, `set_frame_duration`), the process mode and triggers, and its own stream. This is the mission-critical client — e.g. the vision component of a robot controller. |
| **observer** | `/observer` (a trailing slash or query string is tolerated) | Read-only. May discover cameras, query state and limits, and subscribe to frames — with or without payload — but can change nothing. Intended as a CCTV-style telemetry view for humans. |

Clients written before roles existed connect to the bare URL and are therefore commanders; nothing changes for them.

Permitted commands per role:

| Command | commander | observer |
|---|---|---|
| `discover`, `get_state`, `get_frame_duration_limits`, `get_lens_position_limits` | ✓ | ✓ |
| `start_stream`, `stop_stream`, `set_header_only` | ✓ | ✓ (own stream only; see §4.7) |
| `configure`, `unconfigure`, `start_cameras`, `stop_cameras`, `set_process_mode`, `trigger_capture`, `reset_frame_counts`, `set_lens_position`, `set_exposure_time`, `set_frame_duration` | ✓ | ✗ → `error` with `code: "forbidden"` (§7) |

The two streams are **independent**: each role has its own subscription set, its own header-only flag and its own rate control, so the commander can have streaming off while the observer watches, and vice versa. When both stream at once the commander has **priority** (§5.8). The observer's subscription is **persistent** across camera stop/start cycles, and the server **pushes `state` messages** to both clients so an observer can wait indefinitely for a commander to start the cameras without polling (§3.1).

Unsolicited messages by role: the `state` push (§3.1) and the `capture_timeout` error (§7.1) go to both clients; `trigger_result` (§4.17) goes to the commander only.

### Close codes observed/logged
`1000` normal, `1001` going away, `1006` abnormal (no close frame), `1009` message too big, any other treated as unexpected.

## 2. Message Framing

- **Text frames** — JSON commands (client → server) and JSON responses (server → client). Non-text control messages are rejected with an error response.
- **Binary frames** — server → client only. Carry the chunked binary frame protocol (§5). Clients never send binary frames.

All JSON objects the server sends include a `type` field. All JSON commands the client sends include a `cmd` field.

## 3. State Machine

```
                    configure
IDLE ─────────────────────────────────▶ CONFIGURED ──start_cameras──▶ RUNNING
 ▲                                          ▲  │                          │
 │                                          │  │                          │
 └──────────────── unconfigure ─────────────┘  └────── stop_cameras ──────┘
```

- **IDLE** — discovery done, no camera pipeline allocated. Accepts `configure`.
- **CONFIGURED** — camera pipeline configured (libcamera acquired, buffers allocated); not yet capturing. Accepts `start_cameras` (→ RUNNING) or `unconfigure` (→ IDLE, releases pipeline resources).
- **RUNNING** — cameras capturing. `start_stream` / `stop_stream` enable per-camera delivery; `trigger_capture` (in the `trigger` process mode) saves a frame on demand. `stop_cameras` returns to CONFIGURED.

To re-configure with a different `CameraConfig`: `unconfigure` (if CONFIGURED) or `stop_cameras` then `unconfigure` (if RUNNING), then `configure`.

Commands rejected outside the required state return an `error` response but do not change state.

### 3.1 State pushes

Every transition is announced to **both** connected clients with an unsolicited `state` message — the same shape as the `get_state` reply (§4.2) — whose `cause` names the command that caused it (`configure`, `unconfigure`, `start_cameras`, `stop_cameras`). The same message is also pushed to a client when the *other* role connects or disconnects (`cause` = `commander_connected`, `commander_disconnected`, `observer_connected`, `observer_disconnected`), so an observer UI can show whether a commander is present.

The push follows the command's own `status` reply on the commander's socket. A client that issues `get_state` should match the reply by `cause: "query"` rather than by type alone.

```json
{"type": "state", "state": "running", "cause": "start_cameras",
 "role": "observer", "commander_connected": true, "observer_connected": true}
```

## 4. Control Protocol (Text / JSON)

All requests have shape `{"cmd": "<name>", ...}`. All responses have shape `{"type": "<kind>", ...}`.

### 4.1 `discover`
Enumerate cameras whose libcamera `properties::Model` contains `sensor` as a
case-insensitive substring. Allowed in any state.

Camera discovery is cached for the life of the server process, but only once
it succeeds: the **first** `discover` call that matches at least one camera
performs the actual libcamera enumeration/filtering and locks in that
`sensor`, and every later call (even with a different `sensor`) just returns
that same cached camera list. A `discover` call that matches zero cameras
does **not** lock anything in — it re-runs the enumeration on the next call,
so a client can retry with a corrected `sensor` (e.g. after a typo, or before
the camera is attached) without restarting the server.

**Request**
```json
{"cmd": "discover", "params": {
  "sensor": "imx519"
}}
```

`params` is optional. `sensor` defaults to `"imx519"` when omitted (preserves
pre-sensor-param behaviour). Any substring that appears in the attached
sensor's reported model name works, e.g. `"imx477"`, `"ov5647"`.

**Response**
```json
{"type": "discovery", "cameras": [
  {"id": 0, "type": "imx519"},
  {"id": 1, "type": "imx519"}
]}
```

`id` is the stable index used by `start_stream`, `stop_stream`, and in binary frame headers. `type` is the matched camera's actual libcamera `properties::Model` string (not necessarily equal to the requested `sensor` substring). If no attached camera matches, `cameras` is an empty array — this is not an error.

### 4.2 `get_state`
Query the current state-machine state. Allowed in any state. Clients should send this immediately after connecting to synchronise their local state with the server rather than assuming IDLE.

**Request**
```json
{"cmd": "get_state"}
```

**Response**
```json
{"type": "state", "state": "idle", "cause": "query",
 "role": "commander", "commander_connected": true, "observer_connected": false}
```

| Field | Meaning |
|---|---|
| `state` | `"idle"`, `"configured"` or `"running"` |
| `cause` | `"query"` for a reply; the triggering command or connection event for a push (§3.1) |
| `role` | The recipient's own role, `"commander"` or `"observer"` |
| `commander_connected`, `observer_connected` | Whether each role's slot is currently occupied |

### 4.3 `configure`
Apply identical configuration to all cameras. Required state: **IDLE**.

**Request**
```json
{"cmd": "configure", "params": {
  "width": 2328,
  "height": 1748
}}
```

All fields under `params` are optional; omitted fields keep server defaults (see `CameraConfig` in `types.hpp`).

**Output pixel format:** YUV420 (`V4L2_PIX_FMT_YUV420`), reflected in every binary frame header's `pixel_format` FourCC.

**Response (success)** → `status`:
```json
{"type": "status", "message": "Configured: 2328x1748 YUV420"}
```
**Response (failure)** → `error`.

On success the server transitions to **CONFIGURED**.

### 4.4 `unconfigure`
Release the camera pipeline resources (libcamera, buffers, mappings) and return to **IDLE**. Required state: **CONFIGURED**. After `unconfigure`, the client must `configure` again before `start_cameras`.

**Request**
```json
{"cmd": "unconfigure"}
```

**Response (success)** → `status`:
```json
{"type": "status", "message": "Unconfigured: returned to IDLE"}
```
**Response (failure)** → `error` (wrong state, or libcamera resource release failed).

Use `unconfigure` when you want to apply a new `CameraConfig` to the existing server process without disconnecting. The intended flow is `stop_cameras` (if RUNNING) → `unconfigure` → `configure`.

### 4.5 `set_process_mode`
Configure the optional on-device frame-processing pipeline (detection and/or saving). Can be called in any state; takes effect when cameras start.

> Renamed from `set_save_mode` (see changelog). The `mode` values are unchanged; a new `save_frames` param decouples detection from disk writing.

**Request**
```json
{"cmd": "set_process_mode", "mode": "none|buffer|batch|trigger|checkerboard|checkerboard2x2|aruco|aruco2x2", "params": {
  "save_frames": true,
  "output_dir": ".",
  "prepend_timestamp_to_dir": false,
  "batch_size": 10,
  "writer_threads": 4,
  "trigger_skip_frames": 0,
  "checkerboard_rows": 8,
  "checkerboard_cols": 11,
  "checkerboard_full_res_detection": false,
  "checkerboard_num_threads": 4,
  "aruco_full_res_detection": false,
  "aruco_num_threads": 4,
  "aruco_corner_refine": false,
  "backlog_max_bytes": 0,
  "disk_write_bytes_per_sec": 0,
  "allow_overcommit": false
}}
```

`mode` required; `params` and all its members optional.

The mode names **what is done to each captured frame**. Whether processed frames are also written to disk is a separate axis: `save_frames` (default `true`).

| Mode | Behavior |
|---|---|
| `none` | No processing |
| `buffer` | Held in RAM; flushed to disk on `stop_cameras`. Bounded by `backlog_max_bytes` — see [§4.5.1](#451-resource-guards). Nothing is written before the flush, so this mode's entire capacity is that budget. |
| `batch` | Multi-threaded writer; `writer_threads` workers, flushes every `batch_size` frames |
| `trigger` | Nothing is written until the client sends [`trigger_capture`](#417-trigger_capture); each request saves one frame per armed camera through the `writer_threads` pool. Streaming is unaffected — live preview keeps running so the operator can see what is framed. Built for automated calibration rigs: move the camera, wait for it to stop, then trigger, so no saved frame carries motion blur. `trigger_skip_frames` (default `0`) sets the default number of frames to discard per camera before the kept one, overridable per request. |
| `checkerboard` | Detect an OpenCV checkerboard; report its corners to the streaming client (§5.4.1). When `save_frames`, also save frames in which a checkerboard is detected |
| `checkerboard2x2` | Split each frame's Y plane into 4 equal quadrants and detect a checkerboard in each; report the corners of every detecting quadrant. When `save_frames`, save the whole frame if **any** quadrant detects. The Y plane is built the same way as `checkerboard` (full-res or 2x2-subsampled per `checkerboard_full_res_detection`) and then split, so each sub-frame is half-width × half-height of that buffer. The 4 detections run in parallel, batched by `checkerboard_num_threads` (clamped to `[1, 4]`). |
| `aruco` | Detect ArUco/AprilTag markers (`cv::aruco::detectMarkers`, dictionary hard-coded `DICT_APRILTAG_16h5`) on the Y plane (full-res or 2x2-subsampled per `aruco_full_res_detection`); report each marker's **id** and **4 corners** to the streaming client (§5.4.2). When `save_frames`, also save frames in which at least one marker is detected. |
| `aruco2x2` | Split each frame's Y plane into 4 equal quadrants (same construction as `aruco`) and detect markers in each; report every detected marker. When `save_frames`, save the whole frame if **any** quadrant detects. The 4 detections run in parallel, batched by `aruco_num_threads` (clamped to `[1, 4]`). A marker straddling a quadrant boundary may be reported by more than one quadrant, or missed if it lands in no single quadrant intact. |

`save_frames` (default `true`): when `false`, disk writing is disabled while detection still runs and still streams its side-products (corners/markers). In a detector mode this is the **optimized detection-only pipeline** — the writer pool is never started and the per-detection full-frame copy into the write queue is skipped, so `frames_saved` stays `0`. For `buffer`/`batch` (which have no side-product) `false` simply produces no output.

`checkerboard_*` fields apply to `checkerboard` / `checkerboard2x2`; `aruco_*` fields apply to `aruco` / `aruco2x2`; `trigger_skip_frames` applies to `trigger`. `aruco_corner_refine` selects OpenCV's corner-refinement method: `false` ⇒ `CORNER_REFINE_NONE` (fastest, raw quad corners; the real-time default), `true` ⇒ `CORNER_REFINE_SUBPIX` (sub-pixel corners, slower). If `prepend_timestamp_to_dir` is true, a timestamp is prepended to `output_dir`. `output_dir`, `prepend_timestamp_to_dir`, `batch_size`, and `writer_threads` are ignored when `save_frames` is `false`.

All four detector modes (`checkerboard`, `checkerboard2x2`, `aruco`, `aruco2x2`) run detection **best-effort** on a worker thread off the capture path: because detection is CPU-bound (tens–hundreds of ms per frame) it cannot keep up with a high capture rate, so frames that arrive while the detector is busy are skipped (latest-frame-wins into the detector). Streaming is gated on the detector — **only frames the detector actually processed are streamed to the client**, each carrying its detection result (this is independent of `save_frames`). See §5.4 and §5.7.

**Response** → `status` (`"Process mode configured: <mode>"`) or `error` on invalid mode.

#### 4.5.1 Resource guards

A mode that retains every frame (`buffer`) or writes every frame (`batch`) builds
a backlog whenever it outruns its sink. Uncapped, that backlog grows until the
kernel OOM-kills the server mid-capture — and on the way there the swap thrash
starves libcamera's completion thread past the RPi dequeue watchdog, which kills
the capture pipeline permanently (see [§7.1](#71-capture-timeouts)). At
4608x2592 YUV420 one frame is 17.9 MB, so on a 1 GB Pi 5 this takes about
**54 frames** — under half a minute at 2 fps.

Three params bound that, all optional:

| Param | Default | Meaning |
|---|---|---|
| `backlog_max_bytes` | `0` | Ceiling on RAM held by frames captured but not yet written (`buffer`'s store plus the writer pool's queue). `0` ⇒ the server derives one: half of `MemAvailable`, clamped to `[64 MiB, 2 GiB]`. |
| `disk_write_bytes_per_sec` | `0` | Sustained write throughput of `output_dir`'s filesystem, used by the `start_cameras` feasibility check. `0` ⇒ the server measures it once with a 32 MiB `O_DIRECT` probe (adds ~1.5 s to the first `start_cameras` in `batch` mode). |
| `allow_overcommit` | `false` | Start even when the feasibility check says the mode cannot keep up. The check still runs and still logs; it just stops refusing. |

**On reaching the budget** the server keeps the frames it has already accepted
and refuses new ones, so a flush yields a usable contiguous *prefix* of the
capture rather than a scattered set of survivors. It logs one warning and then
counts silently; the total is reported as `frames_dropped_backlog` in the
`stop_cameras` reply (§4.9) and in a `capture_timeout` error (§7.1). A non-zero
count means the recording has a gap.

### 4.6 `start_cameras`
Required state: **CONFIGURED**. Starts frame processing (if non-`none`) and all capture pipelines; transitions to **RUNNING** on success.

```json
{"cmd": "start_cameras"}
```
Response: `status` or `error`.

Beyond the usual wrong-state failure, this **refuses to start** when the
configured mode provably cannot keep up with the camera on this hardware, and
the `error` message names the arithmetic. Two checks run, both skipped when
`allow_overcommit` is set:

- **`batch` vs. disk throughput** — `bytes_per_frame x fps x cameras` against
  the measured (or declared) write rate of `output_dir`. Only `batch` writes
  every captured frame continuously; `trigger` and the detector modes write only
  what is asked for or detected, so neither has a rate to check ahead of time.
- **`buffer` vs. the backlog budget** — refused only when the budget cannot hold
  even two seconds of capture, which is a misconfiguration rather than an
  intent. A budget that holds a short-but-deliberate window is allowed, and
  degrades by dropping frames at the ceiling.

Example refusal:

```json
{
  "type": "error",
  "message": "batch mode at 17.9 MB/frame x 2.00 fps needs 35.8 MB/s, but this filesystem sustains only 21.0 MB/s (1.7x too slow). ..."
}
```

### 4.7 `start_stream`
Subscribe the **calling connection** to binary delivery for one camera. Each role has its own subscription set; this command never affects the other client's stream.

```json
{"cmd": "start_stream", "camera_id": 0, "max_fps": 5, "subsample": 2}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `camera_id` | integer | *(required)* | A camera id from `discover`. |
| `max_fps` | number ≥ 0 | `0` | Cap on the frames per second delivered for this camera; `0` = uncapped. Enforced latest-wins: when the cap next allows a send, the newest available frame is sent, so a capped stream never lags capture. |
| `subsample` | `1`, `2`, `4` or `8` | `1` | Decimate the streamed YUV420 payload by this factor in each dimension (box average of a `subsample × subsample` block; `2` cuts the payload to a quarter). The frame header's `width`/`height`/`bytes_per_line` and any detection block describe the **decimated** frame (§5.3, §5.4). Header-only frames report the decimated geometry too. |

Re-issuing `start_stream` for a camera that is already subscribed updates its options in place.

**Required state:** a **commander** needs **RUNNING**, and its subscriptions are cleared by `stop_cameras` and by its disconnect. An **observer** may subscribe in **any state** — the subscription is **persistent**: it survives `stop_cameras` / `start_cameras` cycles (frames simply stop and resume) and is cleared only by `stop_stream` or the observer's own disconnect. Cameras must have been discovered (by either client) so `camera_id` can be validated.

Response: `status` (`"Started streaming camera 0"`, followed by the non-default options and, for an observer subscribing while the cameras are not running, `" - subscribed; frames will flow once the cameras are running"`) or `error` (unknown `camera_id`, invalid option, commander outside RUNNING).

### 4.8 `stop_stream`
Stop binary delivery for one camera on the calling connection. Cameras keep capturing; the other client's stream is unaffected.

```json
{"cmd": "stop_stream", "camera_id": 0}
```
Response: `status` or `error` (if camera wasn't streaming).

### 4.9 `stop_cameras`
Required state: **RUNNING**. Stops the commander's streaming, stops capture, flushes pending saves, returns to **CONFIGURED**. The observer's subscriptions are kept (§4.7); it resumes on the next `start_cameras`.

```json
{"cmd": "stop_cameras"}
```

**Response** — extended `status`:
```json
{
  "type": "status",
  "message": "All cameras stopped",
  "frames_saved": 1234,
  "bytes_written": 567890123,
  "frames_dropped_backlog": 0
}
```

`frames_saved`, `bytes_written` and `frames_dropped_backlog` are totals for the
capture session that just ended — they are zeroed by `start_cameras`, not
carried across sessions of one server process. `frames_dropped_backlog` counts
frames refused because the backlog budget was full (§4.5.1). Non-zero ⇒
capture outran its sink and the recording has a gap.

### 4.10 `reset_frame_counts`
Zeros `frame_counter`, `frames_dropped` on every camera and `frames_saved` on the saver. Allowed in any state.

```json
{"cmd": "reset_frame_counts"}
```
Response: `status`.

### 4.11 `set_header_only`
Toggle header-only streaming mode **for the calling connection** (each role has its own flag). When enabled, streamed frames carry a chunk header with `total_chunks = 0` and `total_size = 0`, and **no `CHNK` data packets follow**. Useful for timing/metadata probes without transferring pixel bytes.

```json
{"cmd": "set_header_only", "enabled": true}
```
`enabled` defaults to `false` if omitted. Response: `status`.

### 4.12 `set_lens_position`
Set the focus mode. Required state: **CONFIGURED** or **RUNNING** (rejected in IDLE; no camera pipeline exists there).

Single field `lens_position` (number, dioptres):

- **`lens_position < 0`** (e.g. `-1`) — engage continuous autofocus (`AfMode = AfModeContinuous`). This is also the server's default at first start, so a fresh session is already in autofocus without sending this command.
- **`lens_position >= 0`** — manual focus at that lens position (`AfMode = AfModeManual`, `LensPosition = lens_position`). The useful range depends on the attached sensor and its libcamera tuning JSON's `rpi.af` `map` (which clamps the actual usable range); the IMX519 module's is roughly `0.0` (infinity) to `~10.0` (closest macro). The server caps positive values at `32.0` to reject obvious garbage — this cap is a sanity bound, **not** the hardware range; use `get_lens_position_limits` (§4.16) to discover the real `min`/`max`.

The setting is **global** — every discovered camera receives the same value.

**Request**
```json
{"cmd": "set_lens_position", "lens_position": -1}
```
```json
{"cmd": "set_lens_position", "lens_position": 4.5}
```

**Response (success)** → `status` (`"Lens position: continuous autofocus"` or `"Lens position: manual @ <value> dioptres"`).
**Response (failure)** → `error` (wrong state, missing or non-numeric `lens_position`, non-finite, or `> 32.0`).

In **CONFIGURED** the value is stashed and applied at the next `start_cameras` (via libcamera's initial `ControlList`). In **RUNNING** it is attached to the next requeued capture request and takes effect within ~1 frame. The setting persists across `stop_cameras` / `start_cameras` cycles within the same session.

This command only *requests* a focus state. To read back the focus the IPA actually applied to each delivered frame — including the per-frame position chosen by continuous/auto AF — use the `lens_position` and `af_state` fields in the binary frame header (§5.3).

**libcamera prerequisite:** the deployed libcamera tuning JSON for the attached sensor must include the `rpi.af` algorithm block (the apt-installed Pi tuning ships with it for IMX519). Without it, libcamera silently ignores `AfMode` / `LensPosition`. On IMX519, the PDAF parsing patch is optional — without it, continuous AF still works via CDAF, just with slower hunt.

### 4.13 `set_exposure_time`
Set the exposure (shutter) mode. Required state: **CONFIGURED** or **RUNNING** (rejected in IDLE; no camera pipeline exists there).

Single field `exposure_time` (integer, microseconds):

- **`exposure_time < 0`** (e.g. `-1`) — engage auto AE (`ExposureTimeMode = ExposureTimeModeAuto`). This is also the server's default at first start.
- **`exposure_time > 0`** — manual shutter at that duration (`ExposureTimeMode = ExposureTimeModeManual`, `ExposureTime = exposure_time`). Valid range: `[1, 1000000]` µs (1 µs to 1 s).
- **`exposure_time == 0`** — rejected (not a physical shutter duration).

The setting is **global** — every discovered camera receives the same value.

**Request**
```json
{"cmd": "set_exposure_time", "exposure_time": -1}
```
```json
{"cmd": "set_exposure_time", "exposure_time": 10000}
```

**Response (success)** → `status` (`"Exposure time: auto AE"` or `"Exposure time: manual @ <value> µs"`).
**Response (failure)** → `error` (wrong state, missing or non-numeric `exposure_time`, `== 0`, or `> 1000000`).

In **CONFIGURED** the value is stashed and applied at the next `start_cameras` (via libcamera's initial `ControlList`). In **RUNNING** it is attached to the next requeued capture request and takes effect within ~1 frame. The setting persists across `stop_cameras` / `start_cameras` cycles within the same session.

### 4.14 `set_frame_duration`
Lock the per-frame interval (and therefore the framerate) by setting libcamera's `FrameDurationLimits` to `{frame_duration, frame_duration}`. Required state: **CONFIGURED** or **RUNNING** (rejected in IDLE; no camera pipeline exists there).

Single field `frame_duration` (integer, microseconds):

- **`frame_duration <= 0`** — **unset**. At the next `start_cameras` no `FrameDurationLimits` is applied (libcamera defaults — frame interval driven by exposure). When unset mid-stream, the previous lock is released by applying the sensor's hardware max range to the next requeued request.
- **`frame_duration > 0`** — lock both min and max to that value. E.g. `33333` µs ≈ 30 fps, `16667` µs ≈ 60 fps. Valid range: `[1, 1000000000]` µs. libcamera silently clamps to the sensor's hardware-supported range; use `get_frame_duration_limits` to discover it.

The setting is **global** — every discovered camera receives the same value. Server default at first start is **unset**.

**Request**
```json
{"cmd": "set_frame_duration", "frame_duration": 33333}
```
```json
{"cmd": "set_frame_duration", "frame_duration": -1}
```

**Response (success)** → `status` (`"Frame duration: unset"` or `"Frame duration: locked @ <value> µs"`).
**Response (failure)** → `error` (wrong state, missing or non-numeric `frame_duration`, or `> 1000000000`).

In **CONFIGURED** the value is stashed and applied at the next `start_cameras` (via libcamera's initial `ControlList`). In **RUNNING** it is attached to the next requeued capture request and takes effect within ~1 frame. The setting persists across `stop_cameras` / `start_cameras` cycles within the same session.

Note: with manual exposure (`set_exposure_time` > 0), the effective frame rate is bounded by `max(exposure_time, frame_duration)` — libcamera cannot deliver frames faster than the shutter integrates.

### 4.15 `get_frame_duration_limits`
Query the sensor's hardware `FrameDurationLimits` range and the currently-applied lock (if any). Required state: **CONFIGURED** or **RUNNING** (the limits come from libcamera's `ControlInfoMap`, which is only populated after `configure`).

**Request**
```json
{"cmd": "get_frame_duration_limits"}
```

**Response**
```json
{
  "type": "frame_duration_limits",
  "min": 33333,
  "max": 120000000,
  "num_cameras": 4,
  "current": {"min": 33333, "max": 33333}
}
```

| Field | Type | Notes |
|---|---|---|
| `min` | integer | Minimum frame duration in µs as reported by libcamera's `ControlInfoMap` for this camera |
| `max` | integer | Maximum frame duration in µs as reported by libcamera |
| `num_cameras` | integer | Number of logical libcamera cameras currently configured (always `1` with the Arducam quad-board which presents all physical sensors as a single camera) |
| `current` | object \| `null` | `{min, max}` when a lock is in effect (currently `min == max`); `null` when unset |

All discovered cameras on a given server run the same sensor type (selected via the `sensor` param of `discover`, §4.1), so the response carries a single shared range rather than per-camera entries.

**Important — advertised limits vs. achievable rate:** `min` is what libcamera's driver reports as the underlying sensor's hardware capability. With a multi-sensor consolidation board (e.g. the Arducam quad-camera kit), the driver presents all 4 physical sensors as one logical camera and internally manages cycling or synchronising them. This internal overhead means the actual achievable frame duration is **higher than `min`** — empirically, with the Arducam 16MP IMX519 quad-camera kit on Pi 5, `min=33333 µs` (30 fps advertised) yields approximately **15 fps** in practice. Always use `frame_duration_us` from the binary frame header (§5.3) to measure the frame rate actually being delivered at runtime, rather than relying on `min`.

### 4.16 `get_lens_position_limits`
Query the camera's hardware `LensPosition` range — the focus range the IPA advertises. Required state: **CONFIGURED** or **RUNNING** (the limits come from libcamera's `ControlInfoMap`, which is only populated after `configure`).

**Request**
```json
{"cmd": "get_lens_position_limits"}
```

**Response**
```json
{
  "type": "lens_position_limits",
  "min": 0.0,
  "max": 10.0,
  "default": 1.0,
  "num_cameras": 4
}
```

| Field | Type | Notes |
|---|---|---|
| `min` | number \| `null` | Minimum lens position in dioptres (`0.0` = infinity focus) from libcamera's `ControlInfoMap`. `null` when the module has no focuser (fixed focus) or the control is otherwise unavailable. |
| `max` | number \| `null` | Maximum lens position in dioptres (closest macro; closest focus distance ≈ `1 / max` metres). `null` as above. |
| `default` | number \| `null` | The IPA's default lens position in dioptres. `null` if the pipeline advertises no default, or as above. |
| `num_cameras` | integer | Number of logical libcamera cameras currently configured (always `1` with the Arducam quad-board, which presents all physical sensors as a single camera). |

All discovered cameras on a given server run the same sensor type (selected via the `sensor` param of `discover`, §4.1), so the response carries a single shared range rather than per-camera entries.

Unlike `get_frame_duration_limits` (which uses `{0, 0}` to mean "unavailable"), the lens-range fields use JSON `null` for "unavailable", because `0` dioptres (infinity focus) is itself a valid limit. Clients should fall back to a sensible default range when a field is `null`.

### 4.17 `trigger_capture`
Required state: **RUNNING**, and the process mode must be `trigger` (§4.5). Saves one frame per armed camera — the camera shutter-button of the protocol. In any other process mode the request is rejected with an `error` and nothing is saved.

**Request**
```json
{"cmd": "trigger_capture", "camera_id": 0, "skip_frames": 0}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `camera_id` | integer | *(absent)* | Trigger only this camera. When omitted, **every running camera** is armed and the trigger completes when all of them have delivered — the right shape for a multi-camera rig where all views must correspond to the same pose. Errors if the named camera isn't running. |
| `skip_frames` | integer ≥ 0 | `trigger_skip_frames` from `set_process_mode` (itself `0`) | Frames to discard **per camera** before the one that is kept. `0` saves the very next frame each camera delivers. Raise it when the frame already in flight when the trigger arrives may still be blurred — e.g. an arm that has only just come to rest. |

**Response** → `trigger_result`, sent **asynchronously** once every armed camera has delivered its frame and that frame has been handed to the writer pool. This is the signal a robot loop waits on before moving again. Note the frame is queued for writing, not yet flushed to disk, when the ack is sent.

```json
{"type": "trigger_result", "trigger_id": 3, "cancelled": false,
 "captures": [{"camera_id": 0, "frame_id": 412, "filename": "calib/cam0-412.yuv"}]}
```

| Field | Type | Meaning |
|---|---|---|
| `trigger_id` | integer | Monotonic per-session id, starting at `1` and reset by `start_cameras`. Lets a client match acks to requests. |
| `cancelled` | boolean | `true` when capture stopped (`stop_cameras`) while the trigger was still waiting on a camera. `captures` then holds only the cameras that did deliver, so a waiting client is never left hanging. |
| `captures[]` | array | One entry per camera that delivered, **in delivery order** (not `camera_id` order): `camera_id`, the saved frame's `frame_id`, and the `filename` it was queued to. `filename` is empty when the mode runs with `save_frames: false` (capture acknowledged, nothing written). |

Only **one trigger may be outstanding at a time** — a second `trigger_capture` while one is pending returns an `error` (`"A trigger capture is already pending"`) rather than queueing, so the request↔ack correspondence in a robot loop is never ambiguous. Frames arriving while no trigger is armed, and further frames from a camera already served by the pending trigger, are discarded.

Errors: not RUNNING, process mode isn't `trigger`, named camera not running, no running cameras, or a trigger already pending. As with every mode, `set_process_mode` must be sent **before** `start_cameras` — switching into `trigger` on an already-running pipeline leaves the processor in the mode it started with, and `trigger_capture` then errors with a message saying so.

### 4.18 Server response types (summary)

| `type` | Fields | When |
|---|---|---|
| `discovery` | `cameras[]` | Reply to `discover` |
| `state` | `state` (`"idle"` \| `"configured"` \| `"running"`), `cause`, `role`, `commander_connected`, `observer_connected` | Reply to `get_state` (`cause: "query"`); **also unsolicited** on every state transition and when the other role connects/disconnects (§3.1) |
| `status` | `message`, optionally `frames_saved`, `bytes_written`, `frames_dropped_backlog` | Successful command, state change |
| `frame_duration_limits` | `min`, `max`, `current` (`{min, max}` \| `null`) | Reply to `get_frame_duration_limits` |
| `lens_position_limits` | `min`, `max`, `default` (number \| `null`), `num_cameras` | Reply to `get_lens_position_limits` |
| `trigger_result` | `trigger_id`, `cancelled`, `captures[]` | Asynchronous ack for `trigger_capture` — sent when every armed camera has delivered its frame (or the trigger was cancelled by `stop_cameras`) |
| `error` | `message`, optionally `code` and code-specific fields | Bad JSON, unknown command, wrong state, invalid arg, handler exception, infeasible mode (§4.6) |
| `error` (`code: "forbidden"`) | `code`, `message` | An observer sent a command reserved for the commander (§1.1). Nothing changed. |
| `error` (`code: "capture_timeout"`) | `code`, `camera_id`, `stalled_for_us`, `message`, `backlog_bytes`, `backlog_budget_bytes`, `frames_dropped_backlog` | **Unsolicited**, sent to both clients. A camera has stopped delivering frames (§7.1) |

## 5. Binary Frame Protocol

Binary messages are only sent server → client while the receiving connection has at least one camera subscribed and the cameras are running. The commander and the observer each get their own stream — the same captured frame may go to both, under different `frame_uuid`s. Each logical frame is delivered as:

```
1 × [ChunkStartMarker ‖ ChunkHeader]        (single binary WS message)
N × [ChunkData        ‖ payload bytes]       (N binary WS messages, N = total_chunks)
```

All structs are little-endian, packed (`__attribute__((packed))`), no padding. Definitions live in `types.hpp`.

### 5.1 Chunk size

Payload chunks are **32 768 bytes** (`CHUNK_SIZE`) except the last, which carries the remainder. `total_chunks = ceil(total_size / 32768)`.

A chunked transfer that stalls longer than `CHUNK_TIMEOUT` (5 s) is cleaned up server-side.

### 5.2 `ChunkStartMarker` (8 bytes)

| Offset | Field | Type | Value |
|---|---|---|---|
| 0 | `magic` | `uint32` | `0x4348554E` (`'CHUN'`) |
| 4 | `version` | `uint32` | `6` |

### 5.3 `ChunkHeader` (68 bytes, follows the start marker in the same message)

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `frame_uuid` | `uint32` | Random per-frame id; matches `ChunkData.frame_uuid` of all following data packets |
| 4 | `frame_id` | `uint32` | Monotonic capture counter from the camera |
| 8 | `camera_id` | `uint32` | Source camera index |
| 12 | `total_chunks` | `uint32` | Number of `CHNK` packets to follow. **`0` ⇒ header-only frame, no data packets** |
| 16 | `total_size` | `uint32` | Total payload bytes across all chunks. `0` in header-only mode |
| 20 | `bytes_per_line` | `uint32` | Stride of Y plane. Equals `width` on a subsampled stream (tightly packed). |
| 24 | `width` | `uint32` | Pixels — of the **streamed** frame, i.e. the decimated size when the subscription uses `subsample > 1` (§4.7) |
| 28 | `height` | `uint32` | Pixels — likewise |
| 32 | `pixel_format` | `uint32` | V4L2 FourCC (currently `V4L2_PIX_FMT_YUV420` = `'YU12'` = `0x32315559`) |
| 36 | `frames_saved` | `uint32` | Saver's count for this camera at send time |
| 40 | `timestamp_us` | `uint64` | Monotonic hardware capture timestamp in µs (from libcamera `FrameMetadata::timestamp / 1000`). Diff consecutive values for the same `camera_id` to compute actual inter-frame interval (real fps). `0` if unavailable. |
| 48 | `frame_duration_us` | `uint32` | Actual frame duration in µs as reported by the IPA/ISP in libcamera `FrameDuration` metadata. Reflects real per-frame cadence including any multi-camera ISP scheduling overhead. `0` if the metadata was not present. |
| 52 | `corner_block_size` | `uint32` | Size in bytes of the detection block (§5.4) that follows this header in the same WS message. `0` when no block is present. |
| 56 | `num_corner_sets` | `uint16` | Number of records packed in the detection block — `CornerSetHeader` when `detection_kind = 1`, `MarkerSetHeader` when `detection_kind = 2`. `0` when the block is absent. For `checkerboard` this is `0` or `1`; for `checkerboard2x2`, `0..4`; for `aruco` / `aruco2x2`, `0..N` markers. |
| 58 | `reserved` | `uint16` | Reserved for future use; always `0`. |
| 60 | `lens_position` | `float` | Focus distance the libcamera IPA actually applied to this frame, in dioptres (reciprocal metres; `0.0` = infinity). Reported per-frame in manual, auto and continuous AF — the EXIF-style focus record. **`NaN`** when libcamera reported no `LensPosition` for the frame (e.g. a sensor module with no focuser). Check with `isnan` before using; do not treat `0.0` as "unavailable". |
| 64 | `af_state` | `uint8` | libcamera `AfState` for this frame: `0`=Idle, `1`=Scanning, `2`=Focused, `3`=Failed. **`0xFF`** when no `AfState` was reported. Use to tell whether continuous/auto AF had settled (`2`) when `lens_position` was sampled. |
| 65 | `detection_kind` | `uint8` | Classifies the detection block that follows this header (and how to parse each of its `num_corner_sets` records): `0`=none (no detector ran; no block), `1`=checkerboard (records are `CornerSetHeader`, §5.4.1), `2`=aruco (records are `MarkerSetHeader`, §5.4.2). Was part of `reserved2` (always `0`) before v6, so `0` still means "no detection block". |
| 66 | `reserved2` | `uint8[2]` | Padding; always `0`. |

Clients should key reassembly on `frame_uuid` (unique per frame) rather than `frame_id` (per-camera, resets on `reset_frame_counts`).

### 5.4 Detection block (variable, present only when `num_corner_sets > 0`)

Immediately follows `ChunkHeader` in the same WebSocket message. Carries the results produced by the on-device detector in a detector process mode. The `detection_kind` field of `ChunkHeader` (offset 65) selects the per-record format:

- `detection_kind = 1` (checkerboard, §5.4.1) — records are `CornerSetHeader`.
- `detection_kind = 2` (aruco, §5.4.2) — records are `MarkerSetHeader`.

In every case the block is `num_corner_sets` records packed back-to-back, each record being a fixed header followed by `num_corners × { float x, float y }` corner coordinates. Corner coordinates are little-endian IEEE-754, `8` bytes each, in the **streamed frame's Y-plane pixel space** regardless of the saver's `*_full_res_detection` setting or 2x2 quadrant split — clients can overlay them directly on the streamed frame. For a full-resolution stream that is full-frame pixel space; on a subsampled stream (§4.7) the coordinates are divided by the factor along with the frame.

In all four detector modes the streamer only ever sends frames the on-device detector has already processed (see §5.7), so every streamed frame reflects a detection result. When detection found nothing, `num_corner_sets = 0`, `corner_block_size = 0`, and no records follow (`detection_kind` still reports which detector ran). When it found one or more, the block carries them. In the non-detector process modes no detector runs and the block is always absent (`detection_kind = 0`, `num_corner_sets = 0`).

#### 5.4.1 `CornerSetHeader` (checkerboard; `detection_kind = 1`)

One record per detected board:

```
CornerSetHeader (4 bytes)          // set 0
num_corners × { float x, float y } // set 0 corners
CornerSetHeader (4 bytes)          // set 1
num_corners × { float x, float y } // set 1 corners
…
```

`CornerSetHeader` layout (4 bytes):

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `set_id` | `uint8` | `0` for `checkerboard` mode (whole frame). For `checkerboard2x2`, `row * 2 + col` (0..3), where `row` and `col` are `0` for top/left and `1` for bottom/right. |
| 1 | `flags` | `uint8` | Bit 0 set ⇒ coordinates are in full-frame Y-plane pixel space (always `1`). Other bits reserved. |
| 2 | `num_corners` | `uint16` | `= checkerboard_rows × checkerboard_cols` (e.g. `88` for the default 11×8). |

#### 5.4.2 `MarkerSetHeader` (aruco; `detection_kind = 2`)

One record per detected ArUco/AprilTag marker (`aruco` / `aruco2x2` modes; dictionary `DICT_APRILTAG_16h5`):

```
MarkerSetHeader (8 bytes)          // marker 0
4 × { float x, float y }           // marker 0 corners (32 bytes)
MarkerSetHeader (8 bytes)          // marker 1
4 × { float x, float y }           // marker 1 corners
…
```

`MarkerSetHeader` layout (8 bytes):

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `marker_id` | `int32` | The detected marker's id in `DICT_APRILTAG_16h5` (0..29). |
| 4 | `quadrant` | `uint8` | `0` for `aruco` mode (whole frame). For `aruco2x2`, `row * 2 + col` (0..3) — the sub-frame the marker was detected in. The same physical marker may appear in more than one quadrant record. |
| 5 | `flags` | `uint8` | Bit 0 set ⇒ coordinates are in full-frame Y-plane pixel space (always `1`). Other bits reserved. |
| 6 | `num_corners` | `uint16` | Always `4`. |

The 4 corners follow the header in the dictionary's canonical order (clockwise from the marker's top-left). `num_corner_sets` is the number of marker records (one per detected marker), so a single physical marker contributes exactly one record in `aruco`, and up to four across quadrants in `aruco2x2`.

### 5.5 `ChunkData` (16 bytes header, followed by `chunk_size` payload bytes in the same WS message)

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `magic` | `uint32` | `0x43484E4B` (`'CHNK'`) |
| 4 | `frame_uuid` | `uint32` | Must match the preceding `ChunkHeader.frame_uuid` |
| 8 | `chunk_index` | `uint32` | `0 … total_chunks-1`, delivered in order |
| 12 | `chunk_size` | `uint32` | Payload length for this chunk; `≤ CHUNK_SIZE` |
| 16 | payload | bytes | `chunk_size` bytes of YUV420 data |

Concatenating the payloads in `chunk_index` order reproduces the raw frame buffer. Format layout:

- **YUV420 (I420)** planar: `Y` plane (`bytes_per_line × height`), then `U` (`width/2 × height/2`), then `V` (`width/2 × height/2`). Total ≈ `width × height × 3 / 2`.

### 5.6 Multi-camera multiplexing

With multiple cameras streaming, the server uses round-robin scheduling. One frame is delivered atomically (its start marker + header + all `CHNK` packets in sequence) before switching to the next camera. Clients demultiplex on `camera_id`, and should still prefer `frame_uuid` for reassembly.

### 5.7 Backpressure & frame skipping

The server monitors `ws.getBufferedAmount()` with hysteresis (enter at 512 KiB, exit at 128 KiB) and throttles the per-chunk send rate via an adaptive controller. Under sustained pressure the server drops older frames rather than queueing — only the latest frame per camera is retained for transmission. `frame_id` therefore may jump; gaps are expected and not an error.

Header-only mode bypasses the payload path entirely and is unaffected by data backpressure.

Backpressure is tracked **per connection**: the commander's and the observer's sockets have separate buffers, separate hysteresis and separate rate controllers, so a slow observer link never throttles the commander.

**Detector modes.** In `checkerboard` / `checkerboard2x2` / `aruco` / `aruco2x2` an additional gate sits in front of streaming: the frame stream is fed exclusively by the best-effort detector (§4.5). Frames that arrive while the detector is busy are skipped (latest-frame-wins into the detector), and only a frame the detector has finished processing — regardless of whether anything was found — is ever eligible to send. A processed frame may still be dropped afterwards by ordinary backpressure. The net effect is that the streamed frame rate in these modes is bounded by **detector throughput**, not capture rate, so `frame_id` gaps are correspondingly larger and expected.

### 5.8 Streaming priority (commander over observer)

When both clients stream at once over a bandwidth-limited link, the commander's frames come first. The server cannot shape the network, so priority is a rule on what it is willing to **enqueue** for the observer:

> The observer's streamer hands a chunk to its socket only while the commander's connection is **clear**. The commander is clear when it is not consuming payload bandwidth — no commander connected, no camera subscribed, or header-only mode — or when its socket shows no sign of congestion: buffered-but-unsent bytes ≤ 512 KiB (the soft limit of §5.7), its drain handler not reporting backpressure, and **no backpressure event in the last 1 s**.

While the commander is not clear the observer **withholds** new frames (nothing is fetched, so when the commander clears the observer sends the newest frame rather than a stale one — latest-wins) and **pauses mid-frame** between chunks; an observer frame that cannot complete within 5 s is abandoned (the observer's client garbage-collects the incomplete frame, exactly as for a stalled transfer). Observer chunks may still interleave with commander chunks while the commander is uncongested — the rule stops the observer from adding to a stressed link, it does not serialise the two streams.

Consequences: with the commander idle or header-only the observer is unconstrained (except by its own link); with the commander saturating the link the observer's frame rate drops toward zero and the commander sees the same delivery as it would alone. A commander whose *receiver* is slow also reads as congested, so the observer yields conservatively. Observers should use `max_fps` and `subsample` (§4.7) to keep their demand small in the first place.

## 6. Typical Session

```text
client → {"cmd":"discover"}
server → {"type":"discovery","cameras":[{"id":0,"type":"imx519"},{"id":1,"type":"imx519"}]}

client → {"cmd":"configure","params":{"width":2328,"height":1748}}
server → {"type":"status","message":"Configured: 2328x1748 YUV420"}

client → {"cmd":"set_process_mode","mode":"batch","params":{"output_dir":"/data/run1","batch_size":20}}
server → {"type":"status","message":"Process mode configured: batch"}

client → {"cmd":"start_cameras"}
server → {"type":"status","message":"All cameras started successfully"}

client → {"cmd":"start_stream","camera_id":0}
server → {"type":"status","message":"Started streaming camera 0"}
server → <binary: CHUN + header (frame_uuid=A)>
server → <binary: CHNK chunk 0/…>
server → <binary: CHNK chunk 1/…>
...

client → {"cmd":"stop_cameras"}
server → {"type":"status","message":"All cameras stopped","frames_saved":3200,"bytes_written":19327352832}

client → {"cmd":"unconfigure"}
server → {"type":"status","message":"Unconfigured: returned to IDLE"}
```

An observer (connected to `/observer`) alongside that commander:

```text
observer → {"cmd":"discover"}
server   → {"type":"discovery","cameras":[{"id":0,"type":"imx519"},{"id":1,"type":"imx519"}]}
observer → {"cmd":"get_state"}
server   → {"type":"state","state":"idle","cause":"query","role":"observer","commander_connected":true,"observer_connected":true}
observer → {"cmd":"start_stream","camera_id":0,"subsample":2,"max_fps":5}
server   → {"type":"status","message":"Started streaming camera 0 (subsample 2) (max 5 fps) - subscribed; frames will flow once the cameras are running"}
   ... commander configures and starts the cameras ...
server   → {"type":"state","state":"configured","cause":"configure","role":"observer",...}
server   → {"type":"state","state":"running","cause":"start_cameras","role":"observer",...}
server   → <binary: CHUN + header (width/height halved, frame_uuid=B)>
server   → <binary: CHNK ...>
   ... commander stops the cameras: frames pause, the subscription stays ...
server   → {"type":"state","state":"configured","cause":"stop_cameras","role":"observer",...}
   ... commander starts them again: frames resume without a new start_stream ...
observer → {"cmd":"configure","params":{}}
server   → {"type":"error","code":"forbidden","message":"Command 'configure' is not permitted for an observer client (read-only); only the commander may issue it"}
```

## 7. Error Conditions

| Cause | Response |
|---|---|
| Malformed JSON | `error` with `"JSON parse error: …"` |
| Unknown `cmd` | `error` with `"Unknown command: …"` |
| Wrong state for command | `error`, state unchanged |
| Unknown `camera_id` | `error` |
| Invalid `process_mode` value | `error` |
| `trigger_capture` outside the `trigger` process mode | `error` (`"trigger_capture requires the 'trigger' process mode"`), nothing saved |
| `trigger_capture` while one is already pending | `error` (`"A trigger capture is already pending"`) |
| Binary frame sent by client | `error` (`"Only text messages are accepted for commands"`) |
| Observer sends a commander-only command (§1.1) | `error` with `code: "forbidden"`, state unchanged |
| Invalid `start_stream` option (`subsample` not 1/2/4/8, `max_fps` < 0) | `error` |
| Second connection in a role whose slot is taken | HTTP `503` at upgrade (never becomes a WebSocket); the other role's slot is unaffected |
| Mode cannot keep up with the camera | `error` at `start_cameras`, state stays **CONFIGURED** (§4.6) |
| A camera stops delivering frames | **unsolicited** `error` with `code: "capture_timeout"` (§7.1) |

### 7.1 Capture timeouts

Everything else in this section is a reply to something the client sent. This
one is not: the server pushes it whenever a camera stops delivering frames.

libcamera's Raspberry Pi pipeline handler gives up on a stalled frontend after
5x the frame duration (floored at 1 s), logging:

```
WARN  V4L2 /dev/video4[19:cap]: Dequeue timer of 2495000.00us has expired!
ERROR RPI  Camera frontend has timed out!
```

After that, `requestCompleted` is **never called again for the life of that
camera**. libcamera surfaces nothing to the application, so the only evidence is
the absence of frames — which is why an unguarded server presents this as an
indefinite hang on a perfectly healthy WebSocket rather than as a failure.

A watchdog therefore polls every camera and reports one error per failure once a
camera has been silent for **10x its cadence, never less than 3 s**. The cadence
is the slowest of the observed inter-frame interval, the locked
`FrameDurationLimits`, and any manual `ExposureTime` — so deliberately dropping
to a slow frame rate never reads as a stall. The threshold sits above
libcamera's own watchdog by construction, so when this fires the frontend really
is gone.

```json
{
  "type": "error",
  "code": "capture_timeout",
  "camera_id": 0,
  "stalled_for_us": 5712345,
  "message": "Camera 0 has delivered no frame for 5.7 s. The usual cause is ...",
  "backlog_bytes": 304611328,
  "backlog_budget_bytes": 320233472,
  "frames_dropped_backlog": 17
}
```

The error is pushed to **both** clients: the commander is the one that can
recover, and an observer should show the outage rather than a frozen picture.

**Recovery is not automatic**: issue `stop_cameras` then `start_cameras` to
rebuild the pipeline. Reconfiguring is not required, and the camera is left in
its normal **RUNNING** lifecycle state precisely so that path stays open — the
report is a latch, not a state transition.

The latch clears on the next delivered frame, so each stall *episode* is
reported once. A real frontend timeout therefore yields exactly one error (no
frame ever arrives to clear it); a transient stall that recovers on its own
yields one error and then resumes, and may be reported again if it recurs.

The three backlog fields are included because memory exhaustion is the most
common cause on a small-memory host — `backlog_bytes` at the budget with a
non-zero drop count points at the mode's resource demands (§4.5.1) rather than
at the sensor or its cable.

## 8. Versioning

The binary protocol version is carried in `ChunkStartMarker.version` (currently `6`). Clients should reject frames whose `magic` or `version` they don't recognize. There is no explicit version field for the JSON control protocol — backward-incompatible changes will bump the binary version and be documented here.

### Version history

| Version | Change |
|---|---|
| `2` | Baseline: `ChunkHeader` 40 bytes, `ChunkStartMarker` 8 bytes |
| `3` | `ChunkHeader` extended to 52 bytes: added `timestamp_us` (uint64, offset 40) and `frame_duration_us` (uint32, offset 48). `get_frame_duration_limits` response gains `num_cameras` field. |
| `4` | `ChunkHeader` extended to 60 bytes: added `corner_block_size` (uint32, offset 52), `num_corner_sets` (uint16, offset 56), `reserved` (uint16, offset 58). New variable-size `CornerBlock` may follow the header in the same WS message when the save mode is `checkerboard` or `checkerboard2x2` and detection found at least one board on that frame. |
| `5` | `ChunkHeader` extended to 68 bytes: added per-frame focus metadata `lens_position` (float, offset 60; dioptres, `NaN` if unavailable) and `af_state` (uint8, offset 64; libcamera `AfState`, `0xFF` if unavailable), plus `reserved2` (uint8[3], offset 65). Populated from libcamera `LensPosition` / `AfState` request metadata for every frame in manual, auto and continuous AF. Companion text command `get_lens_position_limits` / `lens_position_limits` response added (§4.16) to expose the hardware `LensPosition` range; text-only, no binary change. |
| `6` | `ChunkHeader` stays 68 bytes: the first `reserved2` byte (offset 65) became `detection_kind` (uint8: 0=none, 1=checkerboard, 2=aruco), with `reserved2` shrinking to uint8[2] at offset 66. New `aruco` / `aruco2x2` save modes added (§4.5); when active, the detection block that follows the header uses the new `MarkerSetHeader` record layout (§5.4.2), carrying each marker's `id` (`int32`) and 4 corners. The checkerboard block format (§5.4.1) is unchanged. Old (`detection_kind = 0`) still means "no detection block". |
| `6` (control-protocol only) | New `trigger` process mode plus the `trigger_capture` command and its asynchronous `trigger_result` response (§4.17), and the `trigger_skip_frames` param on `set_process_mode`. Text-only: the binary `ChunkHeader` and detection-block formats are unchanged, so `version` stays `6`. |
| `6` (control-protocol only) | Resource guards added. `set_process_mode` gains `backlog_max_bytes`, `disk_write_bytes_per_sec` and `allow_overcommit` (§4.5.1); `start_cameras` may now refuse an infeasible mode (§4.6); the `stop_cameras` status gains `frames_dropped_backlog` (§4.9); and a new **unsolicited** `error` with `code: "capture_timeout"` reports a camera that has stopped delivering frames (§7.1) — the first server-initiated message that is not a reply or a binary frame. Text-only; `version` stays `6`. |
| `6` (control-protocol only) | **Client roles.** The single client slot became one **commander** (read-write, path `/`) plus one **observer** (read-only, path `/observer`) slot (§1.1); observer write commands fail with `code: "forbidden"`. `state` replies gained `cause`, `role`, `commander_connected`, `observer_connected`, and `state` is now also **pushed** on every transition and on the other role's connect/disconnect (§3.1). `start_stream` gained per-camera `max_fps` and `subsample` options (§4.7) and, for observers, works in any state with a subscription that persists across `stop_cameras`/`start_cameras`; `set_header_only` and backpressure became per connection. Commander-over-observer streaming priority defined (§5.8). On a subsampled stream the header geometry and detection coordinates describe the decimated frame. Binary layout unchanged, so `version` stays `6`. |
| `6` (control-protocol only) | Command `set_save_mode` renamed to `set_process_mode`; status text is now `"Process mode configured: <mode>"` and the invalid-mode error is `Invalid process_mode value` (§4.5). New optional `save_frames` param (default `true`): setting it `false` runs the detector and streams its side-products while writing no frames to disk. `mode` values, the binary `ChunkHeader` (incl. its `frames_saved` field), and the detection block format are all unchanged, so `version` stays `6`. |
