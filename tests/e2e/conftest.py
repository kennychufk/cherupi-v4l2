"""Shared pytest fixtures for end-to-end tests.

The server process is started once per session and reused across tests. Each
test gets a fresh `CherupiClient` that tears the camera pipeline back down to
IDLE on teardown so tests don't leak state into each other.
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Iterator, Optional

import pytest

from .client import CherupiClient


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

    proc = subprocess.Popen(
        [str(binary_path)],
        stdout=subprocess.PIPE,
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
    (if RUNNING) → unconfigure (if CONFIGURED) → set_save_mode(none). Each
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
    try:
        c.unconfigure()
    except Exception:
        pass
    try:
        c.set_save_mode("none")
    except Exception:
        pass
    # Let the sensor pipeline fully release before the next acquire.
    time.sleep(0.5)


@pytest.fixture
def tmp_output_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """A temp directory for save-mode tests.

    NOTE: /tmp is tmpfs on the Pi, and BATCH mode uses O_DIRECT which tmpfs
    doesn't support. Pytest's tmp_path_factory lives under the project's
    pytest-cache dir by default — override with `pytest --basetemp=<path>`
    if the cache ends up on tmpfs too.
    """
    return tmp_path_factory.mktemp("cherupi_e2e_save")


@pytest.fixture
def discovered_cameras(client: CherupiClient) -> list[dict]:
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras discovered — hardware required for e2e")
    return cams


# --- Shared test config -----------------------------------------------------

# Low-resolution configuration used across tests. Keeps per-frame payload
# modest (~1.5 MB per YUV420 frame at 1456x1088) so `batch_size` runs finish
# within the test timeout.
TEST_CONFIG = {
    "width": 1456,
    "height": 1088,
    "crop_width": 4656,
    "crop_height": 3496,
    "crop_left": 8,
    "crop_top": 48,
}


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
