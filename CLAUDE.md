# CLAUDE.md - cherupi-v4l2

## Summary

WebSocket server for Raspberry Pi 5 that streams frames from multiple IMX519 cameras via libcamera. Single-client; binary frames over chunked transfer; optional saving and on-device checkerboard / ArUco (AprilTag) marker detection.

## Layout

```
src/     C++ server sources and headers (flat; all internal includes are quote-form with just the filename)
tools/   Standalone companion binaries (yuv420_convert.cpp)
tests/   unit/ (no hardware), hw/ (live IMX519 libcamera), e2e/ (pytest over the WebSocket protocol); fixtures/ holds test inputs
scripts/ run_e2e.sh — full e2e orchestrator across this repo + sibling telefacet clients
docs/    Protocol spec and design notes
```

## Pipeline

libcamera (Pi5 PiSP frontend) → YUV420 main stream → app
- AWB is handled by libcamera's IPA (not custom).
- Frames carry a FourCC; default is `V4L2_PIX_FMT_YUV420`.
- `tools/yuv420_convert.cpp` is the standalone converter for frames written to disk.

## Components

All paths below are under `src/`.

- `main.cpp` — entry point, signal handling
- `websocket_server.*` — uWebSockets server, command routing, single-client policy, state machine
- `command_parser.*` — pure JSON command parsing + state-machine gate (used by `websocket_server`)
- `camera_manager.*` — multi-camera lifecycle (configure → start → stop)
- `camera.*` — per-camera libcamera wrapper: owns `libcamera::Camera`, `CameraConfiguration`, `FrameBufferAllocator`, request queue, mmap of DMA buffers, frame callbacks
- `stream_manager.*` — round-robin streaming, backpressure/skipping, chunked transfer (writes to an abstract `FrameSink`)
- `rate_controller.*` — `AdaptiveRateController`, used by `stream_manager`
- `frame_sink.hpp` / `uws_frame_sink.hpp` — abstract byte sink + uWebSockets adapter
- `frame_processor.*` / `frame_processor_helpers.*` — `FrameProcessor` runs the per-frame process modes NONE / BUFFER / BATCH / CHECKERBOARD / ARUCO plus pure helpers (filename, timestamped dir, Y-plane extraction)
- `checkerboard_detector.*` — OpenCV `findChessboardCornersSB` detection
- `aruco_detector.*` — OpenCV `cv::aruco::detectMarkers` detection (dictionary hard-coded to `DICT_APRILTAG_16h5`)
- `capacity.*` — resource arithmetic: bytes/frame, the backlog budget derived from `MemAvailable`, an `O_DIRECT` probe of the output filesystem's sustained write rate, and the feasibility verdicts `start_cameras` gates on. Pure except for the two OS probes, so the arithmetic is unit-tested.
- `types.hpp` — logger, protocol constants, `CameraConfig`, `ProcessConfig`, chunk headers

## WebSocket Protocol

Port 9001, single client. Text (JSON) for control, binary (chunked `CHUN` header + N `CHNK` data packets) for frames. YUV420 output; 32 KiB chunks; header-only mode skips payload.

Most commands answer synchronously with `status`/`error`. The one exception is `trigger_capture` (TRIGGER mode), whose `trigger_result` ack is emitted from a capture thread once the triggered frame arrives — marshalled back onto the uWS loop via `UwsFrameSink::sendText`, since uWS sockets are loop-thread-only.

Full specification: [`docs/websocket-protocol.md`](docs/websocket-protocol.md).

**Keep `docs/websocket-protocol.md` in sync with any change to the protocol or client-server behavior** (commands, response shapes, state transitions, binary struct layout, chunk/version constants, error conditions). Update it in the same commit as the code change.

## State Machine

`IDLE → CONFIGURED → RUNNING → CONFIGURED → IDLE`

- `configure` IDLE→CONFIGURED (allocates libcamera resources)
- `start_cameras` CONFIGURED→RUNNING (begins capture)
- `stop_cameras` RUNNING→CONFIGURED (halts capture, keeps pipeline)
- `unconfigure` CONFIGURED→IDLE (releases libcamera resources)

A per-camera `ERROR` state exists but is not currently a state-machine target — and deliberately so. A stalled camera is flagged by a latch in `Camera` (`markCaptureFailed`), not by a state change: `Camera::stop()` early-returns unless the camera is `RUNNING`, so moving a stalled camera to `ERROR` would leave libcamera started with `requestCompleted` still connected and make `stop_cameras` + `start_cameras` — the documented recovery — impossible.

## Process Modes

Set via `set_process_mode`; each mode names what happens to a captured frame. Whether processed frames are also written to disk is an orthogonal axis — `save_frames` (default `true`).

- `NONE` — no processing
- `BUFFER` — held in RAM, flushed to disk on stop. Bounded by `backlog_max_bytes` (see **Resource Guards**); nothing is written before the flush, so that budget is the mode's entire capacity.
- `BATCH` — multi-threaded disk writer (`writer_threads`, `batch_size`)
- `TRIGGER` — save-on-demand: frames stream live but nothing is written until the client sends `trigger_capture`, which saves one frame per armed camera (all running cameras by default, or one via `camera_id`) and acks asynchronously with `trigger_result` once every armed camera has delivered. `skip_frames` (default from `trigger_skip_frames`) discards settling frames first. One trigger outstanding at a time. Built for a robot-arm calibration rig — move, stop, trigger — so saved frames carry no motion blur.
- `CHECKERBOARD` — detect a checkerboard and report its corners to the client (`rows`, `cols`, `full_res_detection`, `num_threads`); when `save_frames`, also save frames where one is detected
- `CHECKERBOARD2X2` — same params; split each frame into 4 equal quadrants, detect in each, report every detecting quadrant's corners; when `save_frames`, save if any quadrant detects. The 4 detections run in parallel, batched by `num_threads` (clamped to `[1, 4]`).
- `ARUCO` — detect ArUco/AprilTag markers (`cv::aruco::detectMarkers`, dictionary hard-coded `DICT_APRILTAG_16h5`) and report each marker's id + 4 corners to the client. Params: `aruco_full_res_detection`, `aruco_corner_refine` (false ⇒ `CORNER_REFINE_NONE`, true ⇒ `CORNER_REFINE_SUBPIX`). When `save_frames`, also save frames where a marker is detected.
- `ARUCO2X2` — same params (`aruco_num_threads` for the quadrant parallelism, clamped `[1, 4]`); split each frame into 4 equal quadrants, detect in each; when `save_frames`, save if any quadrant detects.
- **`save_frames: false`** decouples detection from persistence: the detector still runs and streams its side-products (corners/markers), but no frames are written — the writer pool is never started and the per-detection full-frame copy is skipped (optimized detection-only pipeline). For `buffer`/`batch` it just means no output.
- Output directory optionally prepended with timestamp (ignored when `save_frames: false`).

## Build

CMake 3.14+, C++17. Targets:
- `camera_ws_server` — the server
- `yuv420_convert` — standalone YUV420→image converter for saved frames
- `unit_tests` / `hw_tests` — GoogleTest binaries (ctest labels `unit` / `hardware`); gated by `-DCHERUPI_BUILD_TESTS=ON` (default ON)
- End-to-end pytest suite under `tests/e2e/` — opt in with `-DCHERUPI_BUILD_E2E_TESTS=ON` (ctest label `e2e`, needs `pip install -r tests/e2e/requirements.txt` and live IMX519). See `tests/e2e/README.md`.

`yuv420_convert` has no libcamera/RPi dependency, so it can be built alone on any machine with `-DCHERUPI_TOOLS_ONLY=ON`, which skips the server, uWebSockets/libcamera fetch, and tests (needs only `OpenCV` + pthreads).

### Dependencies
- System: `libcamera` (pkg-config), `OpenCV`, `OpenSSL`, `zlib`, pthreads
- Fetched: `nlohmann/json` v3.11.3, `uSockets` v0.8.8, `uWebSockets` v20.62.0
- Flags: `-O3 -march=armv8.2-a+fp16+simd -mtune=cortex-a76 -ffast-math -funroll-loops -ftree-vectorize`

## Resource Guards

At 4608x2592 YUV420 a frame is **17.9 MB**. A mode that retains every frame
(`BUFFER`) or writes every frame (`BATCH`) builds a backlog whenever it outruns
its sink, and an unbounded backlog does not merely waste memory — it kills
capture:

```
backlog grows -> swap thrash -> libcamera's completion thread misses the RPi
dequeue watchdog (5x frame duration, floored at 1 s) -> "Camera frontend has
timed out!" -> requestCompleted never fires again -> OOM kill
```

On a 1 GB Pi 5 that is ~54 frames, i.e. under half a minute at 2 fps. Three
mechanisms bound it, all configured through `set_process_mode`:

1. **`ProcessConfig::backlog_max_bytes`** caps the RAM held by captured-but-unwritten frames (`buffered_frames` + `write_queue` + `pending_batch`), accounted in `FrameProcessor::reserveBacklog` / `releaseBacklog`. `0` ⇒ half of `MemAvailable`, clamped to `[64 MiB, 2 GiB]`. On reaching it the processor **keeps the frames already accepted and refuses new ones**, so a flush yields a usable contiguous prefix; drops are counted (`getFramesDroppedForBacklog`) and warned once.
2. **A feasibility gate in `start_cameras`** refuses a mode that provably cannot keep up: `BATCH` against the measured write rate of `output_dir`, `BUFFER` against a budget too small to hold two seconds. `allow_overcommit` downgrades the refusal to a warning.
3. **A capture watchdog** (`WebSocketServer::captureWatchdogLoop`, 250 ms ticks) reports a camera that has gone silent for 10x its cadence (min 3 s) as an unsolicited `error` with `code: "capture_timeout"`. libcamera surfaces a frontend timeout to nobody, so without this the server hangs silently on a healthy socket. The cadence is the slowest of observed interval, locked `FrameDurationLimits` and manual `ExposureTime`, so a deliberate slow-down never reads as a stall.

`FrameData` is intentionally rule-of-zero. It used to hand-roll a copy
constructor and copy assignment identical to the compiler's, which suppressed
the implicit *move* operations — so every `std::move(frame)` in the codebase
silently deep-copied a multi-MB payload. Do not reintroduce them.

## Key Design Points

- Single client simplifies state and resource ownership.
- All cameras share one `CameraConfig` (identical resolution).
- Latest-frame-wins for streaming; older frames dropped.
- Adaptive rate control + chunked transfer keeps WebSocket responsive under load.
- Header-only mode lets a client track framing without pulling pixel data.
