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

## Web client e2e suite (telefacet-web)

The sibling `webWs/telefacet-web` repo ships a Vitest suite that imports the
production `src/services/WebSocketManager.js` **unchanged** and runs it against
a live `camera_ws_server` via the `ws` npm package (stubs the browser
`WebSocket` global). Passing it proves wire-level compatibility between the
server and the Vue app without booting a browser.

Two suites live there:

- `test/e2e/websocket-manager.test.js` — live-server tests; needs a Pi with
  IMX519 + running `camera_ws_server`. Mirrors this Python suite's structure
  (lifecycle, header-only, save modes, multi-server).
- `test/e2e/protocol-rejection.test.js` — unit-level; spins an in-process
  `ws` server to validate v1/v2 CHUN handling. No hardware needed.

### Install

```bash
cd ../../webWs/telefacet-web
npm install
```

### Run

Start the server (here), then run the web suite (there):

```bash
# in this repo
./build/camera_ws_server &

# in telefacet-web
cd ../../webWs/telefacet-web
TELEFACET_WS_URL=ws://localhost:9001 npm run test:e2e
```

The unit-level rejection suite can be run standalone (no hardware):

```bash
npm run test:e2e -- protocol-rejection.test.js
```

| Variable | Default | Purpose |
|---|---|---|
| `TELEFACET_WS_URL` | `ws://localhost:9001` | WebSocket URL of the server |

### Via `scripts/run_e2e.sh`

The umbrella script in this repo runs the Python suite plus the web suite (and
the C++ telefacet suite) back-to-back against one server launch:

```bash
TELEFACET_WEB_DIR=../../webWs/telefacet-web ./scripts/run_e2e.sh
```

Each suite is skipped if its source directory or dependencies are missing, so
the script is safe to run on a dev box that only has one of the clients set
up.

### Notes

- Like the Python suite, tests share one server process. Teardown walks the
  state machine back to IDLE (`stop_cameras → unconfigure → set_save_mode=none`);
  skipping `unconfigure` leaks the `CONFIGURED` state into the next test and
  every subsequent `configure` will return `"Cameras must be idle to configure"`.
- libcamera snaps requested resolutions to the nearest supported sensor mode
  (e.g. 1456×1088 → 2328×1748 on IMX519). Don't pin `frame.width`/`height` to
  the requested values — assert structural invariants instead
  (`width/height > 0`, `bytesPerLine >= width`,
  `data.length === bytesPerLine * height * 3 / 2`).
- BUFFER mode only increments `frames_saved` when `stop_cameras` flushes the
  ring buffer, so the per-frame header stays at 0 mid-stream. For a
  live-updating counter use BATCH mode; for a cumulative total read
  `frames_saved` / `bytes_written` from the `stop_cameras` status payload.
- BATCH mode uses O_DIRECT — don't point `output_dir` at `/tmp` on the Pi
  (tmpfs). A path inside the repo (e.g. `process.cwd() + '/.vitest-e2e-save'`)
  works.
