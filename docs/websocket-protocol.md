# cherupi-v4l2 WebSocket Protocol

This document specifies the wire protocol between a client and the `camera_ws_server`. It covers connection, control commands (JSON text), server responses (JSON text), and the binary chunked frame format.

## 1. Connection

| Item | Value |
|---|---|
| URL | `ws://<host>:9001/` (any path matches `/*`) |
| Subprotocol | none |
| Compression | disabled |
| Max payload | 16 MiB |
| Idle timeout | 120 s |
| Max backpressure | 10 MiB |
| Concurrent clients | **1** — additional connections get HTTP `503 Service Unavailable` during upgrade |

On client disconnect the server keeps cameras running (so frame processing can continue) and only releases the client slot.

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
- **RUNNING** — cameras capturing. `start_stream` / `stop_stream` enable per-camera delivery. `stop_cameras` returns to CONFIGURED.

To re-configure with a different `CameraConfig`: `unconfigure` (if CONFIGURED) or `stop_cameras` then `unconfigure` (if RUNNING), then `configure`.

Commands rejected outside the required state return an `error` response but do not change state.

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
{"type": "state", "state": "idle"}
```

`state` is one of `"idle"`, `"configured"`, or `"running"`.

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
{"cmd": "set_process_mode", "mode": "none|buffer|batch|checkerboard|checkerboard2x2|aruco|aruco2x2", "params": {
  "save_frames": true,
  "output_dir": ".",
  "prepend_timestamp_to_dir": false,
  "batch_size": 10,
  "writer_threads": 4,
  "checkerboard_rows": 8,
  "checkerboard_cols": 11,
  "checkerboard_full_res_detection": false,
  "checkerboard_num_threads": 4,
  "aruco_full_res_detection": false,
  "aruco_num_threads": 4,
  "aruco_corner_refine": false
}}
```

`mode` required; `params` and all its members optional.

The mode names **what is done to each captured frame**. Whether processed frames are also written to disk is a separate axis: `save_frames` (default `true`).

| Mode | Behavior |
|---|---|
| `none` | No processing |
| `buffer` | Ring-buffer in RAM; flushed to disk on `stop_cameras` |
| `batch` | Multi-threaded writer; `writer_threads` workers, flushes every `batch_size` frames |
| `checkerboard` | Detect an OpenCV checkerboard; report its corners to the streaming client (§5.4.1). When `save_frames`, also save frames in which a checkerboard is detected |
| `checkerboard2x2` | Split each frame's Y plane into 4 equal quadrants and detect a checkerboard in each; report the corners of every detecting quadrant. When `save_frames`, save the whole frame if **any** quadrant detects. The Y plane is built the same way as `checkerboard` (full-res or 2x2-subsampled per `checkerboard_full_res_detection`) and then split, so each sub-frame is half-width × half-height of that buffer. The 4 detections run in parallel, batched by `checkerboard_num_threads` (clamped to `[1, 4]`). |
| `aruco` | Detect ArUco/AprilTag markers (`cv::aruco::detectMarkers`, dictionary hard-coded `DICT_APRILTAG_16h5`) on the Y plane (full-res or 2x2-subsampled per `aruco_full_res_detection`); report each marker's **id** and **4 corners** to the streaming client (§5.4.2). When `save_frames`, also save frames in which at least one marker is detected. |
| `aruco2x2` | Split each frame's Y plane into 4 equal quadrants (same construction as `aruco`) and detect markers in each; report every detected marker. When `save_frames`, save the whole frame if **any** quadrant detects. The 4 detections run in parallel, batched by `aruco_num_threads` (clamped to `[1, 4]`). A marker straddling a quadrant boundary may be reported by more than one quadrant, or missed if it lands in no single quadrant intact. |

`save_frames` (default `true`): when `false`, disk writing is disabled while detection still runs and still streams its side-products (corners/markers). In a detector mode this is the **optimized detection-only pipeline** — the writer pool is never started and the per-detection full-frame copy into the write queue is skipped, so `frames_saved` stays `0`. For `buffer`/`batch` (which have no side-product) `false` simply produces no output.

`checkerboard_*` fields apply to `checkerboard` / `checkerboard2x2`; `aruco_*` fields apply to `aruco` / `aruco2x2`. `aruco_corner_refine` selects OpenCV's corner-refinement method: `false` ⇒ `CORNER_REFINE_NONE` (fastest, raw quad corners; the real-time default), `true` ⇒ `CORNER_REFINE_SUBPIX` (sub-pixel corners, slower). If `prepend_timestamp_to_dir` is true, a timestamp is prepended to `output_dir`. `output_dir`, `prepend_timestamp_to_dir`, `batch_size`, and `writer_threads` are ignored when `save_frames` is `false`.

All four detector modes (`checkerboard`, `checkerboard2x2`, `aruco`, `aruco2x2`) run detection **best-effort** on a worker thread off the capture path: because detection is CPU-bound (tens–hundreds of ms per frame) it cannot keep up with a high capture rate, so frames that arrive while the detector is busy are skipped (latest-frame-wins into the detector). Streaming is gated on the detector — **only frames the detector actually processed are streamed to the client**, each carrying its detection result (this is independent of `save_frames`). See §5.4 and §5.7.

**Response** → `status` (`"Process mode configured: <mode>"`) or `error` on invalid mode.

### 4.6 `start_cameras`
Required state: **CONFIGURED**. Starts frame processing (if non-`none`) and all capture pipelines; transitions to **RUNNING** on success.

```json
{"cmd": "start_cameras"}
```
Response: `status` or `error`.

### 4.7 `start_stream`
Required state: **RUNNING**. Begin binary delivery for one camera.

```json
{"cmd": "start_stream", "camera_id": 0}
```
Response: `status` or `error` (e.g. unknown `camera_id`, cameras not running).

### 4.8 `stop_stream`
Stop binary delivery for one camera. Cameras keep capturing.

```json
{"cmd": "stop_stream", "camera_id": 0}
```
Response: `status` or `error` (if camera wasn't streaming).

### 4.9 `stop_cameras`
Required state: **RUNNING**. Stops streaming, stops capture, flushes pending saves, returns to **CONFIGURED**.

```json
{"cmd": "stop_cameras"}
```

**Response** — extended `status`:
```json
{
  "type": "status",
  "message": "All cameras stopped",
  "frames_saved": 1234,
  "bytes_written": 567890123
}
```

### 4.10 `reset_frame_counts`
Zeros `frame_counter`, `frames_dropped` on every camera and `frames_saved` on the saver. Allowed in any state.

```json
{"cmd": "reset_frame_counts"}
```
Response: `status`.

### 4.11 `set_header_only`
Toggle header-only streaming mode. When enabled, streamed frames carry a chunk header with `total_chunks = 0` and `total_size = 0`, and **no `CHNK` data packets follow**. Useful for timing/metadata probes without transferring pixel bytes.

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

### 4.17 Server response types (summary)

| `type` | Fields | When |
|---|---|---|
| `discovery` | `cameras[]` | Reply to `discover` |
| `state` | `state` (`"idle"` \| `"configured"` \| `"running"`) | Reply to `get_state` |
| `status` | `message`, optionally `frames_saved`, `bytes_written` | Successful command, state change |
| `frame_duration_limits` | `min`, `max`, `current` (`{min, max}` \| `null`) | Reply to `get_frame_duration_limits` |
| `lens_position_limits` | `min`, `max`, `default` (number \| `null`), `num_cameras` | Reply to `get_lens_position_limits` |
| `error` | `message` | Bad JSON, unknown command, wrong state, invalid arg, handler exception |

## 5. Binary Frame Protocol

Binary messages are only sent server → client while at least one camera is streaming. Each logical frame is delivered as:

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
| 20 | `bytes_per_line` | `uint32` | Stride of Y plane |
| 24 | `width` | `uint32` | Pixels |
| 28 | `height` | `uint32` | Pixels |
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

In every case the block is `num_corner_sets` records packed back-to-back, each record being a fixed header followed by `num_corners × { float x, float y }` corner coordinates. Corner coordinates are little-endian IEEE-754, `8` bytes each, in **full-frame Y-plane pixel space** regardless of the saver's `*_full_res_detection` setting or 2x2 quadrant split — clients can overlay them directly on the streamed frame.

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

**Detector modes.** In `checkerboard` / `checkerboard2x2` / `aruco` / `aruco2x2` an additional gate sits in front of streaming: the frame stream is fed exclusively by the best-effort detector (§4.5). Frames that arrive while the detector is busy are skipped (latest-frame-wins into the detector), and only a frame the detector has finished processing — regardless of whether anything was found — is ever eligible to send. A processed frame may still be dropped afterwards by ordinary backpressure. The net effect is that the streamed frame rate in these modes is bounded by **detector throughput**, not capture rate, so `frame_id` gaps are correspondingly larger and expected.

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

## 7. Error Conditions

| Cause | Response |
|---|---|
| Malformed JSON | `error` with `"JSON parse error: …"` |
| Unknown `cmd` | `error` with `"Unknown command: …"` |
| Wrong state for command | `error`, state unchanged |
| Unknown `camera_id` | `error` |
| Invalid `process_mode` value | `error` |
| Binary frame sent by client | `error` (`"Only text messages are accepted for commands"`) |
| Second client connection attempt | HTTP `503` at upgrade (never becomes a WebSocket) |

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
| `6` (control-protocol only) | Command `set_save_mode` renamed to `set_process_mode`; status text is now `"Process mode configured: <mode>"` and the invalid-mode error is `Invalid process_mode value` (§4.5). New optional `save_frames` param (default `true`): setting it `false` runs the detector and streams its side-products while writing no frames to disk. `mode` values, the binary `ChunkHeader` (incl. its `frames_saved` field), and the detection block format are all unchanged, so `version` stays `6`. |
