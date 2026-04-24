# cherupi-v4l2

WebSocket server for Raspberry Pi 5 that streams frames from multiple IMX519 cameras via libcamera. Single client, YUV420 output, optional frame saving, optional on-device checkerboard detection.

## Install dependencies

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config \
    libcamera-dev libopencv-dev libssl-dev zlib1g-dev
```

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j4
```

CMake fetches `nlohmann/json`, `uSockets`, `uWebSockets`, and (for tests) GoogleTest automatically.

Targets:
- `camera_ws_server` — the server
- `yuv420_convert` — converts saved YUV420 frames to images
- `unit_tests` / `hw_tests` — run `ctest -L unit` or `ctest -L hardware`

## Run

```bash
sudo ./camera_ws_server
```

Listens on `ws://0.0.0.0:9001`. Requires root (or appropriate permissions) for libcamera device access.

## Project layout

```
src/      Server sources
tools/    yuv420_convert (C++) and test_client.py
tests/    unit/ and hw/ GoogleTest suites
docs/     Protocol spec and design notes
```

## Quick start with the test client

```bash
pip install websocket-client
python3 tools/test_client.py --host localhost --port 9001
```

This connects, discovers cameras, configures at 2328×1748, starts streaming for 3 seconds, and prints a line per frame.

## WebSocket protocol (overview)

Control messages are JSON; frames are sent as chunked binary messages.

```json
{"cmd": "discover"}
{"cmd": "configure", "params": {"width": 2328, "height": 1748}}
{"cmd": "set_save_mode", "mode": "none|buffer|batch|checkerboard", "params": {...}}
{"cmd": "start_cameras"}
{"cmd": "start_stream", "camera_id": 0}
{"cmd": "stop_stream",  "camera_id": 0}
{"cmd": "stop_cameras"}
```

State machine: `IDLE → CONFIGURED → RUNNING → CONFIGURED`. Commands are rejected when issued in the wrong state.

Full specification, including binary header layout and chunk framing, lives in [`docs/websocket-protocol.md`](docs/websocket-protocol.md).

## Save modes

Set with `set_save_mode` before `start_cameras`:

- `none` — stream only
- `buffer` — in-memory ring
- `batch` — multi-threaded disk writer (`writer_threads`, `batch_size`)
- `checkerboard` — detect and save only frames containing a checkerboard (`checkerboard_rows`, `checkerboard_cols`, `checkerboard_full_res_detection`, `checkerboard_num_threads`)

Common parameters: `output_dir`, `prepend_timestamp_to_dir`. Saved YUV420 frames can be converted to images with `yuv420_convert`.

## Limitations

- One client connection at a time.
- All cameras share the same configuration.
- Cameras must be configured once per connection, before starting.
- Save mode must be set before `start_cameras`.
