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

On client disconnect the server keeps cameras running (so frame saving can continue) and only releases the client slot.

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
Enumerate cameras. Allowed in any state.

**Request**
```json
{"cmd": "discover"}
```

**Response**
```json
{"type": "discovery", "cameras": [
  {"id": 0, "type": "IMX519"},
  {"id": 1, "type": "IMX519"}
]}
```

`id` is the stable index used by `start_stream`, `stop_stream`, and in binary frame headers.

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
  "height": 1748,
  "crop_width": 4656,
  "crop_height": 3496,
  "crop_left": 8,
  "crop_top": 48,
  "awb": {
    "enabled": true,
    "interval": 10,
    "speed": 0.05,
    "warmup_frames": 10
  }
}}
```

All fields under `params` and `params.awb` are optional; omitted fields keep server defaults (see `CameraConfig` / `AwbConfig` in `types.hpp`).

`awb.*` is accepted for protocol compatibility only — libcamera's IPA owns AWB at runtime and these fields are not applied. Clients may omit the `awb` object.

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

### 4.5 `set_save_mode`
Configure the optional on-device frame saver. Can be called in any state; takes effect when cameras start.

**Request**
```json
{"cmd": "set_save_mode", "mode": "none|buffer|batch|checkerboard", "params": {
  "output_dir": ".",
  "prepend_timestamp_to_dir": false,
  "batch_size": 10,
  "writer_threads": 4,
  "checkerboard_rows": 8,
  "checkerboard_cols": 11,
  "checkerboard_full_res_detection": false,
  "checkerboard_num_threads": 4
}}
```

`mode` required; `params` and all its members optional.

| Mode | Behavior |
|---|---|
| `none` | Saving disabled |
| `buffer` | Ring-buffer in RAM; flushed to disk on `stop_cameras` |
| `batch` | Multi-threaded writer; `writer_threads` workers, flushes every `batch_size` frames |
| `checkerboard` | Only save frames in which an OpenCV checkerboard is detected |

`checkerboard_*` fields apply only to `checkerboard` mode. If `prepend_timestamp_to_dir` is true, a timestamp is prepended to `output_dir`.

**Response** → `status` (`"Save mode configured: <mode>"`) or `error` on invalid mode.

### 4.6 `start_cameras`
Required state: **CONFIGURED**. Starts frame saver (if non-`none`) and all capture pipelines; transitions to **RUNNING** on success.

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
- **`lens_position >= 0`** — manual focus at that lens position (`AfMode = AfModeManual`, `LensPosition = lens_position`). Typical IMX519 useful range is roughly `0.0` (infinity) to `~10.0` (closest macro); the libcamera tuning JSON's `rpi.af` `map` clamps the actual usable range. The server caps positive values at `32.0` to reject obvious garbage.

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

**libcamera prerequisite:** the deployed libcamera tuning JSON for IMX519 must include the `rpi.af` algorithm block (the apt-installed Pi tuning ships with it). Without it, libcamera silently ignores `AfMode` / `LensPosition`. The IMX519 PDAF parsing patch is optional — without it, continuous AF still works via CDAF, just with slower hunt.

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
  "min": 33,
  "max": 120000000,
  "current": {"min": 33333, "max": 33333}
}
```

| Field | Type | Notes |
|---|---|---|
| `min` | integer | Sensor hardware minimum frame duration in µs |
| `max` | integer | Sensor hardware maximum frame duration in µs |
| `current` | object \| `null` | `{min, max}` when a lock is in effect (currently `min == max`); `null` when unset |

All discovered cameras run the same sensor (IMX519), so the response carries a single shared range rather than per-camera entries.

### 4.16 Server response types (summary)

| `type` | Fields | When |
|---|---|---|
| `discovery` | `cameras[]` | Reply to `discover` |
| `state` | `state` (`"idle"` \| `"configured"` \| `"running"`) | Reply to `get_state` |
| `status` | `message`, optionally `frames_saved`, `bytes_written` | Successful command, state change |
| `frame_duration_limits` | `min`, `max`, `current` (`{min, max}` \| `null`) | Reply to `get_frame_duration_limits` |
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
| 4 | `version` | `uint32` | `2` |

### 5.3 `ChunkHeader` (40 bytes, follows the start marker in the same message)

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

Clients should key reassembly on `frame_uuid` (unique per frame) rather than `frame_id` (per-camera, resets on `reset_frame_counts`).

### 5.4 `ChunkData` (16 bytes header, followed by `chunk_size` payload bytes in the same WS message)

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `magic` | `uint32` | `0x43484E4B` (`'CHNK'`) |
| 4 | `frame_uuid` | `uint32` | Must match the preceding `ChunkHeader.frame_uuid` |
| 8 | `chunk_index` | `uint32` | `0 … total_chunks-1`, delivered in order |
| 12 | `chunk_size` | `uint32` | Payload length for this chunk; `≤ CHUNK_SIZE` |
| 16 | payload | bytes | `chunk_size` bytes of YUV420 data |

Concatenating the payloads in `chunk_index` order reproduces the raw frame buffer. Format layout:

- **YUV420 (I420)** planar: `Y` plane (`bytes_per_line × height`), then `U` (`width/2 × height/2`), then `V` (`width/2 × height/2`). Total ≈ `width × height × 3 / 2`.

### 5.5 Multi-camera multiplexing

With multiple cameras streaming, the server uses round-robin scheduling. One frame is delivered atomically (its start marker + header + all `CHNK` packets in sequence) before switching to the next camera. Clients demultiplex on `camera_id`, and should still prefer `frame_uuid` for reassembly.

### 5.6 Backpressure & frame skipping

The server monitors `ws.getBufferedAmount()` with hysteresis (enter at 512 KiB, exit at 128 KiB) and throttles the per-chunk send rate via an adaptive controller. Under sustained pressure the server drops older frames rather than queueing — only the latest frame per camera is retained for transmission. `frame_id` therefore may jump; gaps are expected and not an error.

Header-only mode bypasses the payload path entirely and is unaffected by data backpressure.

## 6. Typical Session

```text
client → {"cmd":"discover"}
server → {"type":"discovery","cameras":[{"id":0,"type":"IMX519"},{"id":1,"type":"IMX519"}]}

client → {"cmd":"configure","params":{"width":2328,"height":1748}}
server → {"type":"status","message":"Configured: 2328x1748 YUV420"}

client → {"cmd":"set_save_mode","mode":"batch","params":{"output_dir":"/data/run1","batch_size":20}}
server → {"type":"status","message":"Save mode configured: batch"}

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
| Invalid `save_mode` value | `error` |
| Binary frame sent by client | `error` (`"Only text messages are accepted for commands"`) |
| Second client connection attempt | HTTP `503` at upgrade (never becomes a WebSocket) |

## 8. Versioning

The binary protocol version is carried in `ChunkStartMarker.version` (currently `2`). Clients should reject frames whose `magic` or `version` they don't recognize. There is no explicit version field for the JSON control protocol — backward-incompatible changes will bump the binary version and be documented here.
