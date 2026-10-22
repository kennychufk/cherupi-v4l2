# CLAUDE.md - cherupi-v4l2

## Summary

WebSocket server for Raspberry Pi 5 that streams frames from multiple IMX519 cameras via libcamera. Two client roles — one read-write **commander** and one read-only **observer** at a time; binary frames over chunked transfer; optional saving and on-device checkerboard / ArUco (AprilTag) marker detection.

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
- `websocket_server.*` — uWebSockets server, command routing, the two client-role slots (commander/observer, chosen by URL path), per-role `ClientSession`s, state pushes, state machine
- `command_parser.*` — pure JSON command parsing, role-from-path, role permission gate, state-machine gate, `start_stream` option parsing (used by `websocket_server`)
- `camera_manager.*` — multi-camera lifecycle (configure → start → stop)
- `camera.*` — per-camera libcamera wrapper: owns `libcamera::Camera`, `CameraConfiguration`, `FrameBufferAllocator`, request queue, mmap of DMA buffers, frame callbacks
- `sensor_mode.*` — pure choice of the native sensor mode (raw format + bit depth) for the configured size, from the formats libcamera advertises on the Raw stream; bit depth varies per mode (IMX477: 10-bit at 1332x990, 12-bit above)
- `focus_region.*` — pure geometry of the fixed block libcamera's RPi IPA averages into the per-frame `FocusFoM` (middle half of the 8x8 PiSP CDAF grid on both axes), reported by `configure` / `get_focus_fom_region`
- `stream_manager.*` — one streaming session per client role: subscriptions with per-camera `StreamOptions` (`max_fps`, `subsample`), round-robin, backpressure/skipping, chunked transfer (writes to an abstract `FrameSink`), plus the FIFO of capture frames (`enqueueCaptures`) sent ahead of live frames. The observer's session yields to the commander's (`setYieldTo`).
- `stream_priority.hpp` — the pure commander-over-observer yield rule (`isClear`), unit-tested
- `frame_subsample.*` — pure YUV420 box-average decimation + corner scaling for the `subsample` stream option
- `rate_controller.*` — `AdaptiveRateController`, used by `stream_manager`
- `frame_sink.hpp` / `uws_frame_sink.hpp` — abstract byte sink + uWebSockets adapter
- `frame_processor.*` / `frame_processor_helpers.*` — `FrameProcessor` runs the per-frame process modes NONE / BUFFER / BATCH / CHECKERBOARD / ARUCO plus pure helpers (filename, timestamped dir, Y-plane extraction)
- `checkerboard_detector.*` — OpenCV `findChessboardCornersSB` detection + `cornerSubPix` refinement on the full-res image
- `aruco_detector.*` — OpenCV `cv::aruco::detectMarkers` detection (dictionary hard-coded to `DICT_APRILTAG_16h5`)
- `raw_stack.*` — pure raw-stacking pieces: 2x2-binned accumulation of 16-bit raw frames, the mean image, the exposure/gain settle check, detection on a stacked float image
- `raw_stacker.*` — `RawStacker`, the `stack_capture` orchestrator: per-camera settle → skip → sum state machine fed from the capture thread, and a worker thread that forms the images, acks, detects and hands them over as capture frames
- `trigger_capturer.*` — `TriggerCapturer`, the `trigger_capture` orchestrator: arms cameras, copies each one's next (post-`skip_frames`) live frame, acks from the capture thread, and runs the mode's detection on a worker before handing the frames over
- `capture_frame.hpp` — `CaptureFrame`, one trigger/stack frame on its way to the commander (frame, detection, `send_image`)
- `capacity.*` — resource arithmetic: bytes/frame, the backlog budget derived from `MemAvailable`, an `O_DIRECT` probe of the output filesystem's sustained write rate, and the feasibility verdicts `start_cameras` gates on. Pure except for the two OS probes, so the arithmetic is unit-tested.
- `types.hpp` — logger, protocol constants, `ClientRole`, `StreamOptions`, `CameraConfig`, `ProcessConfig`, chunk headers

## WebSocket Protocol

Port 9001. Text (JSON) for control, binary (chunked `CHUN` header + N `CHNK` data packets) for frames. YUV420 output; 32 KiB chunks; header-only mode skips payload.

**Client roles.** The URL path picks the role at upgrade: `/observer` ⇒ observer, anything else ⇒ commander. One slot each (503 otherwise). The commander owns the lifecycle, attributes, process mode and triggers; the observer may only `discover`, `get_state`, `get_*_limits`, `get_focus_fom_region`, `start_stream`/`stop_stream`/`set_header_only` (its own stream) — anything else is an `error` with `code: "forbidden"`. The two streams are independent (own subscriptions, header-only flag, backpressure, rate control). Observer subscriptions are persistent: allowed in any state, kept across `stop_cameras`/`start_cameras`. `state` is pushed to both clients on every transition and when the other role connects/disconnects, with `cause`, `role`, `commander_connected`, `observer_connected`. `start_stream` takes per-camera `max_fps` and `subsample` (1/2/4/8).

**Per-frame focus metadata.** Each `ChunkHeader` (v8, 72 bytes) carries the libcamera `LensPosition`, `AfState` and `FocusFoM` the IPA reported for that frame. `FocusFoM` is averaged over a fixed central block the server cannot change; its rectangle rides on the `configure` reply and is queryable with `get_focus_fom_region` (observer-allowed).

**Streaming priority.** The observer's streamer enqueues chunks only while the commander is *clear*: no commander, none of its cameras streaming (or header-only) and no capture frames waiting, or (`bufferedAmount ≤ 512 KiB` and no backpressure in the last 1 s). It withholds new frames (latest-wins) and pauses mid-frame otherwise, abandoning a frame after 5 s. See `stream_priority.hpp` and protocol §5.8.

**On-demand captures.** `trigger_capture` (one live frame per armed camera) and `stack_capture` (a raw stack, needs `raw`) are commands, not modes: they run in any process mode, get the mode's detection, save nothing, and take `send_image`. After the async ack (`trigger_result` / `stack_result`) their frames (`frame_kind` 2 / 1, `capture_id` = trigger/stack id in the header) go to the commander only — queued FIFO ahead of live frames, never latest-wins, whatever the subscription, `subsample`, `max_fps` or header-only state (`send_image` decides pixels). One capture of either kind at a time, from request until its frames are sent (`WebSocketServer::captureInProgress`); `stop_cameras` cancels one still waiting on cameras.

Most commands answer synchronously with `status`/`error`. The exceptions are `trigger_capture`, whose `trigger_result` ack is emitted from a capture thread once the triggered frames arrive, `stack_capture`'s `stack_result` from the stacker's worker, the `state` pushes, and the `capture_timeout` error — all marshalled onto the uWS loop via `UwsFrameSink::sendText`, since uWS sockets are loop-thread-only.

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

Set via `set_process_mode`; each mode names the post-processing applied to a captured frame. Whether processed frames are also written to disk is an orthogonal axis — `save_frames` (default `true`). On-demand captures are not modes (see **On-demand captures** above); their frames get the mode's detection but are never saved.

- `NONE` — no processing
- `BUFFER` — held in RAM, flushed to disk on stop. Bounded by `backlog_max_bytes` (see **Resource Guards**); nothing is written before the flush, so that budget is the mode's entire capacity.
- `BATCH` — multi-threaded disk writer (`writer_threads`, `batch_size`)
- `CHECKERBOARD` — detect a checkerboard and report its corners to the client (`rows`, `cols`, `full_res_detection`, `num_threads`, `subpix_window`); when `save_frames`, also save frames where one is detected. Detected corners are refined with `cv::cornerSubPix` on the full-res Y plane (`checkerboard_subpix_window`, default 15 px half-size, clamped to 40 % of the corner spacing, 0 = off): SB's own corners scatter ~0.12 px and are biased on defocused corners; refined ~0.025 px, for ~4 ms vs ~400 ms detection on a Pi 5 at 4056x3040
- `CHECKERBOARD2X2` — same params; split each frame into 4 equal quadrants, detect in each, report every detecting quadrant's corners; when `save_frames`, save if any quadrant detects. The 4 detections run in parallel, batched by `num_threads` (clamped to `[1, 4]`).
- `ARUCO` — detect ArUco/AprilTag markers (`cv::aruco::detectMarkers`, dictionary hard-coded `DICT_APRILTAG_16h5`) and report each marker's id + 4 corners to the client. Params: `aruco_full_res_detection`, `aruco_corner_refine` (false ⇒ `CORNER_REFINE_NONE`, true ⇒ `CORNER_REFINE_SUBPIX`), `aruco_edge_refine` (default true: after OpenCV, each marker's corners are re-derived from its EDGES in linear light — OpenCV's corners, NONE and SUBPIX alike, sit ~1 px inside the true corner of a blurred square on gamma-encoded luma, which read every ~95 px tag 2 % small on the arm cameras; a marker the fit cannot hold keeps OpenCV's corners) and `aruco_edge_gamma` (the decode exponent, 2.2 as measured on the Pi ISP output; 1.0 = none). When `save_frames`, also save frames where a marker is detected.
- `ARUCO2X2` — same params (`aruco_num_threads` for the quadrant parallelism, clamped `[1, 4]`); split each frame into 4 equal quadrants, detect in each; when `save_frames`, save if any quadrant detects.
- **`save_frames: false`** decouples detection from persistence: the detector still runs and streams its side-products (corners/markers), but no frames are written — the writer pool is never started and the per-detection full-frame copy is skipped (optimized detection-only pipeline). For `buffer`/`batch` it just means no output.
- Output directory optionally prepended with timestamp (ignored when `save_frames: false`).

## Raw Frame Stacking

`configure` with `raw: true` adds the sensor's raw stream, unpacked to 16 bits (SBGGR16: no lossy PISP_COMP1), alongside YUV420. `stack_capture` (commander, RUNNING, any process mode) optionally pins `exposure_time` / `analogue_gain` (gain defaults to 1.0 with an exposure), waits until the frame metadata shows them (≤ 30 frames), skips `skip_frames`, then sums `num_frames` raw frames per 2x2 Bayer quad on the capture thread (`Camera::onRawFrame`, read in place from the DMA buffer under `DMA_BUF_IOCTL_SYNC`), restores the preview settings and replies with an async `stack_result`, then sends each image to the commander as a capture frame (`frame_kind = 1`, pixel format `'SF32'`, float32 half-resolution linear DN above black; corners in its own pixels, `b ↦ 2b + 0.5` to full res). Detection is the process mode's (`checkerboard_subpix_window` halved for the binned image): SB / `detectMarkers` on a gamma-2.2 8-bit rendering, `cornerSubPix` / the ArUco edge fit on the float stack. `stop_cameras` cancels a stack still summing.

Why raw: the ISP's temporal denoise correlates consecutive YUV frames, so stacking YUV barely helps; raw frames are independent and scatter falls as 1/√N (IMX477, 50 ms, gain 1: 0.0154 px → 0.0029 px over 32 frames).

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

**Large buffers go back to the kernel.** `main` pins glibc's mmap threshold at
1 MiB (`mallopt(M_MMAP_THRESHOLD)`). glibc's default threshold rises after the
first large free, after which multi-MB frame buffers come from per-thread heaps
that rarely shrink: the server sat at 516 MB resident while idle on a 1 GB Pi
(27 MB with the fix). The cameras also skip copying out YUV frames nobody uses
— no process mode, no subscription, no trigger waiting — which is what keeps
stacking at full resolution affordable (`WebSocketServer::updateLiveFrames`,
`Camera::setLiveFrames`).

**Frames are shared, not copied, between consumers.** `Camera` publishes each
captured frame as a `shared_ptr<const FrameData>` with a monotonic sequence
number, and `FrameProcessor` does the same for detector-processed frames; each
streaming session keeps its own cursor and takes a reference (latest-wins per
consumer). The sequence counters are never reset, so cursors survive
`stop_cameras`/`start_cameras` and slot clears. `Camera::stop()` drops the
published frame so a late subscriber never receives a stale pre-stop frame.

`FrameData` is intentionally rule-of-zero. It used to hand-roll a copy
constructor and copy assignment identical to the compiler's, which suppressed
the implicit *move* operations — so every `std::move(frame)` in the codebase
silently deep-copied a multi-MB payload. Do not reintroduce them.

## Key Design Points

- One commander owns all mutable state; the observer is strictly read-only, so ownership stays as simple as the single-client design was.
- All cameras share one `CameraConfig` (identical resolution).
- Latest-frame-wins for streaming, per consumer; older frames dropped.
- Adaptive rate control + chunked transfer keeps WebSocket responsive under load.
- Header-only mode lets a client track framing without pulling pixel data.
