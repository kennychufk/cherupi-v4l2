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
- `frame_saver.*` / `frame_saver_helpers.*` — save modes NONE / BUFFER / BATCH / CHECKERBOARD / ARUCO plus pure helpers (filename, timestamped dir, Y-plane extraction)
- `checkerboard_detector.*` — OpenCV `findChessboardCornersSB` detection
- `aruco_detector.*` — OpenCV `cv::aruco::detectMarkers` detection (dictionary hard-coded to `DICT_APRILTAG_16h5`)
- `types.hpp` — logger, protocol constants, `CameraConfig`, `SaveConfig`, chunk headers

## WebSocket Protocol

Port 9001, single client. Text (JSON) for control, binary (chunked `CHUN` header + N `CHNK` data packets) for frames. YUV420 output; 32 KiB chunks; header-only mode skips payload.

Full specification: [`docs/websocket-protocol.md`](docs/websocket-protocol.md).

**Keep `docs/websocket-protocol.md` in sync with any change to the protocol or client-server behavior** (commands, response shapes, state transitions, binary struct layout, chunk/version constants, error conditions). Update it in the same commit as the code change.

## State Machine

`IDLE → CONFIGURED → RUNNING → CONFIGURED → IDLE`

- `configure` IDLE→CONFIGURED (allocates libcamera resources)
- `start_cameras` CONFIGURED→RUNNING (begins capture)
- `stop_cameras` RUNNING→CONFIGURED (halts capture, keeps pipeline)
- `unconfigure` CONFIGURED→IDLE (releases libcamera resources)

A per-camera `ERROR` state exists but is not currently a state-machine target.

## Save Modes

- `NONE` — no save
- `BUFFER` — in-memory ring
- `BATCH` — multi-threaded disk writer (`writer_threads`, `batch_size`)
- `CHECKERBOARD` — save only frames where a checkerboard is detected (`rows`, `cols`, `full_res_detection`, `num_threads`)
- `CHECKERBOARD2X2` — same params; split each frame into 4 equal quadrants and save if any quadrant detects a checkerboard. The 4 detections run in parallel, batched by `num_threads` (clamped to `[1, 4]`).
- `ARUCO` — save only frames where at least one ArUco/AprilTag marker is detected (`cv::aruco::detectMarkers`, dictionary hard-coded `DICT_APRILTAG_16h5`). Params: `aruco_full_res_detection`, `aruco_corner_refine` (false ⇒ `CORNER_REFINE_NONE`, true ⇒ `CORNER_REFINE_SUBPIX`). Each detected marker's id + 4 corners are reported to the streaming client.
- `ARUCO2X2` — same params (`aruco_num_threads` for the quadrant parallelism, clamped `[1, 4]`); split each frame into 4 equal quadrants and save if any quadrant detects a marker.
- Output directory optionally prepended with timestamp.

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

## Key Design Points

- Single client simplifies state and resource ownership.
- All cameras share one `CameraConfig` (identical resolution).
- Latest-frame-wins for streaming; older frames dropped.
- Adaptive rate control + chunked transfer keeps WebSocket responsive under load.
- Header-only mode lets a client track framing without pulling pixel data.
