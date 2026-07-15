"""Disconnect / reconnect / single-client policy.

These tests exercise the server's behaviour when the client goes away
unexpectedly: abrupt TCP drop without a WebSocket close frame, leaving the
server to detect the loss via EOF and run `cleanupConnection`.

Server contract being verified (see `WebSocketServer::cleanupConnection`):

  * `system_state` is preserved across a disconnect — cameras keep capturing
    so frame saving can continue uninterrupted.
  * `streaming_cameras` is cleared (the sink is gone), so a reconnecting
    client must re-issue `start_stream`.
  * The single-client slot is released; a new client can take over.
  * Concurrent second client at upgrade time gets HTTP 503 and never becomes
    a WebSocket.

Each test does its own teardown because the conftest `client` fixture is not
involved — most of these need a fresh, controllable connection.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Iterator

import pytest
import websocket

from .client import CherupiClient, CommandError


# --- helpers ----------------------------------------------------------------


def _retry_connect(c: CherupiClient, timeout: float = 5.0) -> None:
    """Try `c.connect()` until it succeeds or the deadline fires.

    The server's close handler runs on the uWS event loop after EOF, so the
    client slot frees asynchronously after an abrupt drop.
    """
    deadline = time.monotonic() + timeout
    last: BaseException | None = None
    while time.monotonic() < deadline:
        try:
            c.connect()
            return
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(0.1)
    raise TimeoutError(f"Could not reconnect within {timeout}s: {last!r}")


def _drive_to_idle(server_url: str) -> None:
    """Best-effort: connect, walk state back to IDLE, disconnect."""
    c = CherupiClient(server_url)
    try:
        _retry_connect(c, timeout=5.0)
    except TimeoutError:
        return
    try:
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
            c.set_process_mode("none")
        except Exception:
            pass
    finally:
        c.close()
    # Same release window the conftest uses between tests.
    time.sleep(0.5)


@pytest.fixture
def fresh_url(server_url: str) -> Iterator[str]:
    """Yields the server URL; guarantees server is back at IDLE on teardown.

    Disconnect tests don't use the standard `client` fixture (they need to
    abrupt-close themselves), so we install a dedicated cleanup hook.
    """
    yield server_url
    _drive_to_idle(server_url)


# --- single-client policy ---------------------------------------------------


def test_second_client_rejected_with_503(
    client: CherupiClient,
    server_url: str,
) -> None:
    """While one client is connected, a second upgrade gets HTTP 503."""
    # `client` is the slot-holder — touch it once so pyright sees it used.
    assert client._ws is not None
    second = CherupiClient(server_url, open_timeout=2.0)
    with pytest.raises(websocket.WebSocketBadStatusException) as exc_info:
        second.connect()
    assert exc_info.value.status_code == 503


def test_slot_releases_after_graceful_close(fresh_url: str) -> None:
    """After a clean close, a fresh client can connect."""
    a = CherupiClient(fresh_url)
    a.connect()
    a.discover()  # prove it's a working session
    a.close()
    # No abrupt path here — graceful close should free the slot quickly.
    b = CherupiClient(fresh_url)
    _retry_connect(b, timeout=3.0)
    try:
        b.discover()
    finally:
        b.close()


def test_slot_releases_after_abrupt_close(fresh_url: str) -> None:
    """After a TCP drop with no WS close frame (1006), the slot frees."""
    a = CherupiClient(fresh_url)
    a.connect()
    a.discover()
    a.abrupt_close()
    b = CherupiClient(fresh_url)
    _retry_connect(b, timeout=5.0)
    try:
        b.discover()
    finally:
        b.close()


# --- state preservation across disconnect -----------------------------------


def test_abrupt_disconnect_in_configured_preserves_state(fresh_url: str) -> None:
    """Abrupt drop in CONFIGURED: state survives; new client can start_cameras."""
    a = CherupiClient(fresh_url)
    a.connect()
    a.discover()
    a.configure(width=1456, height=1088)
    a.abrupt_close()

    b = CherupiClient(fresh_url)
    _retry_connect(b, timeout=5.0)
    try:
        # Still CONFIGURED → configure must error.
        with pytest.raises(CommandError):
            b.configure(width=1456, height=1088)
        # start_cameras must succeed without re-configuring.
        b.start_cameras()
        b.stop_cameras()
        b.unconfigure()
    finally:
        b.close()


def test_abrupt_disconnect_during_streaming_preserves_state(fresh_url: str) -> None:
    """Abrupt drop while RUNNING: state survives; new client can resume."""
    a = CherupiClient(fresh_url)
    a.connect()
    cams = a.discover()
    if not cams:
        pytest.skip("No cameras")
    cam_id = cams[0]["id"]
    a.configure(width=1456, height=1088)
    a.start_cameras()
    a.start_stream(cam_id)
    a.next_frame(timeout=5.0)  # prove streaming is healthy
    a.abrupt_close()

    b = CherupiClient(fresh_url)
    _retry_connect(b, timeout=5.0)
    try:
        # Still RUNNING → configure must error.
        with pytest.raises(CommandError):
            b.configure(width=1456, height=1088)
        # streaming_cameras was cleared on disconnect; we must re-issue.
        b.start_stream(cam_id)
        frame = b.next_frame(timeout=5.0)
        assert frame.camera_id == cam_id
        # stop_cameras must succeed (proves we are RUNNING).
        b.stop_cameras()
        b.unconfigure()
    finally:
        b.close()


def test_save_continues_during_disconnect(
    fresh_url: str, tmp_output_dir: Path
) -> None:
    """BATCH save mode keeps writing frames while no client is connected.

    `cleanupConnection` deliberately leaves cameras running so the saver's
    frame callback keeps firing. We verify by counting frames_saved on a
    `stop_cameras` issued by a *different* client that connects after the
    original client dropped.
    """
    out = tmp_output_dir / "disconnect_batch"
    out.mkdir()

    a = CherupiClient(fresh_url)
    a.connect()
    cams = a.discover()
    if not cams:
        pytest.skip("No cameras")
    cam_id = cams[0]["id"]
    a.configure(width=1456, height=1088)
    a.set_process_mode("batch", output_dir=str(out), batch_size=5, writer_threads=2)
    a.start_cameras()
    a.start_stream(cam_id)
    # Confirm streaming is up before we drop.
    a.next_frame(timeout=5.0)
    a.abrupt_close()

    # Saving must continue while we are gone.
    disconnect_window = 2.0
    time.sleep(disconnect_window)

    b = CherupiClient(fresh_url)
    _retry_connect(b, timeout=5.0)
    try:
        stop = b.stop_cameras()
        # ~30 fps for ~2 s = ~60 frames; floor very low to keep the test
        # robust on a busy CI Pi. The point is "non-zero and clearly grew".
        assert stop["frames_saved"] >= 10, (
            f"Expected save to continue during disconnect, got "
            f"frames_saved={stop['frames_saved']}"
        )
        assert stop["bytes_written"] > 0
        b.unconfigure()
    finally:
        b.close()


# --- many cycles ------------------------------------------------------------


def test_many_connect_disconnect_cycles(fresh_url: str) -> None:
    """Repeated abrupt disconnects don't leak the client slot or wedge the server.

    The server's close handler runs asynchronously on the uWS event loop after
    EOF, so each cycle gives the slot up to 5 s to free. Half the cycles use
    graceful close, half use abrupt close, to exercise both code paths.
    """
    cycles = 8
    for i in range(cycles):
        c = CherupiClient(fresh_url)
        _retry_connect(c, timeout=5.0)
        try:
            c.discover()
            if i % 2 == 0:
                c.abrupt_close()
            else:
                c.close()
        finally:
            # Belt-and-braces: if the test logic raised, drop the socket so the
            # next iteration is not blocked by a still-occupied slot.
            try:
                c.abrupt_close()
            except Exception:
                pass
        # Brief pause to let the server close handler run before the next
        # iteration's upgrade attempt — avoids a tight reconnect race.
        time.sleep(0.05)

    # Final sanity check: a normal session still works end-to-end.
    final = CherupiClient(fresh_url)
    _retry_connect(final, timeout=5.0)
    try:
        cams = final.discover()
        assert isinstance(cams, list)
    finally:
        final.close()
