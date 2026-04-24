# End-to-End Tests

Black-box pytest suite that drives a running `camera_ws_server` through the v2
WebSocket protocol. Covers:

- Full lifecycle (discover → configure → start → stream → stop)
- State-machine gates and error responses
- Single-client enforcement (second connection gets 503)
- CHUN/CHNK binary frame format (header fields, chunk reassembly, frame_uuid uniqueness)
- Header-only mode
- Save modes: NONE / BUFFER / BATCH / CHECKERBOARD

All tests require **live IMX519 hardware** connected to the Pi 5. There is no
fake-camera backend in the server (by design).

## Install

```bash
pip install -r tests/e2e/requirements.txt
```

## Run

Easiest — let pytest spawn the server:

```bash
pytest tests/e2e/ --server-binary=build/camera_ws_server -v
```

Or start the server yourself and point the tests at it:

```bash
build/camera_ws_server &
pytest tests/e2e/ --server-url=ws://localhost:9001 -v
kill %1
```

Run a single test file:

```bash
pytest tests/e2e/test_binary_protocol.py -v
```

## Via ctest

When the server is built with `-DCHERUPI_BUILD_E2E_TESTS=ON`, the suite is
registered as a ctest entry with label `e2e`:

```bash
cmake -B build -DCHERUPI_BUILD_E2E_TESTS=ON
cmake --build build
ctest --test-dir build -L e2e --output-on-failure
```

The ctest entry passes `--server-binary=build/camera_ws_server` automatically.

## Options

| Option | Default | Purpose |
|---|---|---|
| `--server-binary=PATH` | *(none)* | If set, pytest spawns it and tears down. |
| `--server-url=URL` | `ws://localhost:9001` | Where to connect. |
| `--server-startup-timeout=SECS` | `10` | Seconds to wait for the port to open. |

## Notes

- BATCH mode uses O_DIRECT; pytest's default `--basetemp` is under
  `.pytest_cache` which usually sits on a real filesystem. If you override
  `--basetemp` to tmpfs, BATCH tests may fail.
- `/tmp` is tmpfs on the Pi — avoid it for save-mode output dirs.
- Tests share a single server instance across the session (fast) and reset
  state (stop_cameras, set_save_mode=none, set_header_only=false) between tests.
