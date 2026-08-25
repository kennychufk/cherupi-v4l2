"""Shared pytest fixtures for end-to-end tests.

The server process is started once per session and reused across tests. Each
test gets a fresh `CherupiClient` that tears the camera pipeline back down to
IDLE on teardown so tests don't leak state into each other.
"""

from __future__ import annotations

import os
import shutil
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Iterator, Optional

import pytest

from . import client as client_module
from .client import ROLE_OBSERVER, CherupiClient


DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9001


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--server-binary",
        action="store",
        default=None,
        help="Path to camera_ws_server. If omitted, tests assume one is already "
        "running on --server-url (env CHERUPI_SERVER_URL).",
    )
    parser.addoption(
        "--server-url",
        action="store",
        default=f"ws://{DEFAULT_HOST}:{DEFAULT_PORT}",
        help="WebSocket URL the client connects to.",
    )
    parser.addoption(
        "--server-startup-timeout",
        action="store",
        type=float,
        default=10.0,
        help="Seconds to wait for the server port to open after spawn.",
    )
    parser.addoption(
        "--server-log",
        action="store",
        default=os.environ.get("CHERUPI_SERVER_LOG", ".e2e-server.log"),
        help="File the spawned server's stdout/stderr is written to "
        "(env CHERUPI_SERVER_LOG). Only used with --server-binary.",
    )
    parser.addoption(
        "--width",
        action="store",
        type=int,
        default=int(os.environ.get("CHERUPI_WIDTH", "2328")),
        help="Capture width the suite configures (env CHERUPI_WIDTH). Must be a "
        "native sensor mode of the attached module — the server pins the sensor "
        "mode, so e.g. 1536 (imx708) or 2328 (imx519).",
    )
    parser.addoption(
        "--height",
        action="store",
        type=int,
        default=int(os.environ.get("CHERUPI_HEIGHT", "1748")),
        help="Capture height the suite configures (env CHERUPI_HEIGHT); see --width.",
    )
    parser.addoption(
        "--save-dir",
        action="store",
        default=os.environ.get("CHERUPI_SAVE_DIR", ".e2e-save"),
        help="Directory the save-mode tests point the server's output_dir at "
        "(env CHERUPI_SAVE_DIR). Must be on a real filesystem on the server's "
        "host: BATCH writes with O_DIRECT, and /tmp is tmpfs on the Pi.",
    )
    parser.addoption(
        "--sensor",
        action="store",
        default=os.environ.get("CHERUPI_SENSOR", "imx519"),
        help="Sensor model substring passed to `discover` (env CHERUPI_SENSOR). "
        "Default imx519; use e.g. imx708 for a Camera Module 3 rig.",
    )


# --- Helpers ----------------------------------------------------------------


def _wait_for_port(
    host: str, port: int, timeout: float, proc: Optional[subprocess.Popen] = None
) -> None:
    deadline = time.monotonic() + timeout
    last_err: Optional[BaseException] = None
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            raise RuntimeError(
                f"Server exited with code {proc.returncode} before port {port} opened"
            )
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return
        except OSError as e:
            last_err = e
            time.sleep(0.1)
    raise TimeoutError(
        f"Port {host}:{port} did not open within {timeout}s (last: {last_err!r})"
    )


def _parse_host_port(url: str) -> tuple[str, int]:
    # ws://host:port(/...)  -> (host, port)
    if not url.startswith(("ws://", "wss://")):
        raise ValueError(f"Expected ws(s):// URL, got {url!r}")
    rest = url.split("://", 1)[1].split("/", 1)[0]
    if ":" in rest:
        host, port_s = rest.rsplit(":", 1)
        return host, int(port_s)
    return rest, 443 if url.startswith("wss://") else 80


# --- Fixtures ---------------------------------------------------------------


@pytest.fixture(scope="session")
def server_url(pytestconfig: pytest.Config) -> str:
    return pytestconfig.getoption("--server-url")


@pytest.fixture(scope="session", autouse=True)
def default_sensor(pytestconfig: pytest.Config) -> str:
    """Make every `discover()` in the suite name the rig's sensor."""
    sensor = pytestconfig.getoption("--sensor")
    client_module.DEFAULT_SENSOR = sensor
    return sensor


@pytest.fixture(scope="session", autouse=True)
def ws_server(
    pytestconfig: pytest.Config, server_url: str
) -> Iterator[Optional[subprocess.Popen]]:
    """Spawn camera_ws_server (if --server-binary given) and tear down."""
    binary = pytestconfig.getoption("--server-binary")
    host, port = _parse_host_port(server_url)
    startup_timeout = pytestconfig.getoption("--server-startup-timeout")

    if binary is None:
        # Caller is expected to have started the server externally.
        _wait_for_port(host, port, timeout=startup_timeout)
        yield None
        return

    binary_path = Path(binary).expanduser().resolve()
    if not binary_path.is_file():
        pytest.skip(f"camera_ws_server not found at {binary_path}")

    # The server logs a line every few frames per camera. Piping that into a
    # buffer nobody drains fills the 64 KiB pipe within a minute of streaming,
    # after which every log write blocks the uWS loop thread and the server
    # looks hung (upgrades time out, commands go unanswered). Send it to a
    # file instead — which is also the only way to read it after a failure.
    log_path = Path(pytestconfig.getoption("--server-log")).expanduser()
    log_file = open(log_path, "w")  # noqa: SIM115 — closed in the finally
    proc = subprocess.Popen(
        [str(binary_path)],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        # Fresh process group so we can kill children.
        preexec_fn=os.setsid,
    )
    try:
        _wait_for_port(host, port, timeout=startup_timeout, proc=proc)
        yield proc
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        log_file.close()


@pytest.fixture
def client(server_url: str) -> Iterator[CherupiClient]:
    """A connected client scoped to one test. Resets state on teardown."""
    c = CherupiClient(server_url)
    c.connect()
    try:
        yield c
    finally:
        _reset_server_state(c)
        c.close()


def _reset_server_state(c: CherupiClient) -> None:
    """Drive the server back to IDLE so the next test starts fresh.

    Walk the state machine backwards: set_header_only(false) → stop_cameras
    (if RUNNING) → unconfigure (if CONFIGURED) → set_process_mode(none). Each
    step may fail legitimately (e.g. stop_cameras is only valid in RUNNING),
    so we swallow CommandError at each step — the intent is best-effort
    cleanup, not state assertion.

    A small sleep after unconfigure gives libcamera/V4L2 time to fully
    release the sensor before the next test reconfigures; without it the
    IMX519 pipeline intermittently times out on the next start_cameras.
    """
    # Tests may have abrupt-closed the socket before we got here; in that
    # state every step below will raise something other than CommandError.
    # Each step is best-effort, so swallow broadly.
    try:
        c.set_header_only(False)
    except Exception:
        pass
    try:
        c.stop_cameras()
    except Exception:
        pass
    # Camera settings persist for the life of the server process, so undo any
    # frame-rate lock a test applied (must happen while still CONFIGURED).
    try:
        c.command({"cmd": "set_frame_duration", "frame_duration": -1})
    except Exception:
        pass
    try:
        c.unconfigure()
    except Exception:
        pass
    try:
        c.set_process_mode("none")
    except Exception:
        pass
    # Let the sensor pipeline fully release before the next acquire.
    time.sleep(0.5)


@pytest.fixture
def observer(server_url: str) -> Iterator[CherupiClient]:
    """A connected read-only observer client (protocol §1), scoped to one test.

    Holds the observer slot only; it never changes server state, so no reset
    is needed on teardown beyond closing the socket.
    """
    o = CherupiClient(server_url, role=ROLE_OBSERVER)
    o.connect()
    try:
        yield o
    finally:
        o.close()


@pytest.fixture
def tmp_output_dir(request: pytest.FixtureRequest,
                   pytestconfig: pytest.Config) -> Iterator[Path]:
    """A scratch directory for save-mode tests, removed after each test.

    Deliberately NOT pytest's tmp_path: that lives under /tmp, which is tmpfs
    (RAM) on the Pi. BATCH writes with O_DIRECT, and a run's worth of frames
    (hundreds of MB, retained across pytest's last three basetemp dirs) fills
    a 1 GB Pi's tmpfs — after which every write fails, leaving empty files
    and frames_saved == 0. `--save-dir` defaults to ./.e2e-save on the real
    filesystem, and each test's subdirectory is deleted on teardown so the
    disk does not fill either.
    """
    base = Path(pytestconfig.getoption("--save-dir")).expanduser().resolve()
    leaf = "".join(c if c.isalnum() or c in "-_" else "_" for c in request.node.name)
    path = base / leaf
    shutil.rmtree(path, ignore_errors=True)
    path.mkdir(parents=True)
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


@pytest.fixture
def discovered_cameras(client: CherupiClient) -> list[dict]:
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras discovered — hardware required for e2e")
    return cams


# --- Shared test config -----------------------------------------------------

# Capture geometry used across tests. The server pins the sensor mode to the
# requested size, so it must be a native mode of the attached module (the
# defaults are the IMX519's 2328x1748; pass --width/--height or CHERUPI_WIDTH/
# CHERUPI_HEIGHT for another module, e.g. 1536x864 on an imx708). Updated in
# place by pytest_configure so every `from .conftest import TEST_CONFIG` sees
# the values in force.
TEST_CONFIG = {
    "width": 2328,
    "height": 1748,
}


def pytest_configure(config: pytest.Config) -> None:
    TEST_CONFIG["width"] = config.getoption("--width")
    TEST_CONFIG["height"] = config.getoption("--height")


@pytest.fixture
def configured_client(
    client: CherupiClient, discovered_cameras: list[dict]
) -> CherupiClient:
    """Client that has already done discover + configure (ready for start_cameras).

    Teardown (`_reset_server_state`) drives the server back to IDLE via
    `unconfigure`, so each test starts from a deterministic state.
    """
    client.configure(**TEST_CONFIG)
    return client


@pytest.fixture
def running_client(configured_client: CherupiClient) -> CherupiClient:
    """Client with cameras in RUNNING state (no streams started yet)."""
    configured_client.start_cameras()
    return configured_client
