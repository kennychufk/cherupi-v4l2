"""Commander / observer client roles (protocol §1, §4.19).

A server holds one *commander* (read-write) and one *observer* (read-only)
connection. These tests exercise the slots, the observer's permission gate,
the pushed `state` messages, persistent observer subscriptions across camera
restarts, per-connection header-only, and the observer-only stream options
(`subsample`, `max_fps`).
"""

from __future__ import annotations

import time
from typing import Iterator

import pytest
import websocket

from .client import ROLE_OBSERVER, CherupiClient, CommandError
from .conftest import TEST_CONFIG


def _retry_connect(c: CherupiClient, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    last: BaseException | None = None
    while time.monotonic() < deadline:
        try:
            c.connect()
            return
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(0.1)
    raise TimeoutError(f"Could not connect within {timeout}s: {last!r}")


def _next_frame_matching(c: CherupiClient, predicate, attempts: int = 8,
                         timeout: float = 5.0):
    """Frames already in flight when an option changes may still carry the
    old shape; allow a few frames for the change to take effect."""
    last = None
    for _ in range(attempts):
        last = c.next_frame(timeout=timeout)
        if predicate(last):
            return last
    raise AssertionError(f"No frame matched after {attempts} frames; last={last}")


# --- slots ------------------------------------------------------------------


def test_state_reply_carries_role_and_presence(client: CherupiClient) -> None:
    st = client.get_state()
    assert st["type"] == "state"
    assert st["cause"] == "query"
    assert st["role"] == "commander"
    assert st["commander_connected"] is True
    assert st["observer_connected"] is False
    assert st["state"] in ("idle", "configured", "running")


def test_observer_and_commander_connect_concurrently(
    client: CherupiClient, observer: CherupiClient
) -> None:
    st = client.get_state()
    assert st["role"] == "commander"
    assert st["commander_connected"] is True
    assert st["observer_connected"] is True

    st = observer.get_state()
    assert st["role"] == "observer"
    assert st["cause"] == "query"
    assert st["commander_connected"] is True
    assert st["observer_connected"] is True


def test_second_observer_rejected_with_503(
    server_url: str, observer: CherupiClient
) -> None:
    assert observer.get_state()["observer_connected"] is True
    second = CherupiClient(server_url, role=ROLE_OBSERVER, open_timeout=2.0)
    with pytest.raises(websocket.WebSocketBadStatusException) as exc_info:
        second.connect()
    assert exc_info.value.status_code == 503


def test_commander_slot_is_independent_of_the_observer_slot(
    server_url: str, observer: CherupiClient
) -> None:
    """An observer does not occupy the commander slot, and vice versa."""
    a = CherupiClient(server_url)
    a.connect()
    try:
        assert a.get_state()["observer_connected"] is True
        b = CherupiClient(server_url, open_timeout=2.0)
        with pytest.raises(websocket.WebSocketBadStatusException) as exc_info:
            b.connect()
        assert exc_info.value.status_code == 503
    finally:
        a.close()


def test_observer_slot_releases_after_close(server_url: str) -> None:
    a = CherupiClient(server_url, role=ROLE_OBSERVER)
    a.connect()
    a.get_state()
    a.close()
    b = CherupiClient(server_url, role=ROLE_OBSERVER)
    _retry_connect(b)
    try:
        assert b.get_state()["role"] == "observer"
    finally:
        b.close()


# --- permissions ------------------------------------------------------------


WRITE_COMMANDS = [
    {"cmd": "configure", "params": TEST_CONFIG},
    {"cmd": "unconfigure"},
    {"cmd": "set_process_mode", "mode": "none"},
    {"cmd": "start_cameras"},
    {"cmd": "stop_cameras"},
    {"cmd": "trigger_capture"},
    {"cmd": "reset_frame_counts"},
    {"cmd": "set_lens_position", "lens_position": -1},
    {"cmd": "set_exposure_time", "exposure_time": -1},
    {"cmd": "set_frame_duration", "frame_duration": -1},
]


@pytest.mark.parametrize("cmd", WRITE_COMMANDS, ids=lambda c: c["cmd"])
def test_observer_write_commands_are_forbidden(
    client: CherupiClient, observer: CherupiClient, cmd: dict
) -> None:
    before = client.get_state()["state"]
    with pytest.raises(CommandError) as exc_info:
        observer.command(cmd)
    assert exc_info.value.code == "forbidden"
    assert "observer" in str(exc_info.value)
    # Nothing changed.
    assert client.get_state()["state"] == before


def test_observer_read_commands_are_allowed(observer: CherupiClient) -> None:
    cams = observer.discover()
    assert isinstance(cams, list)
    assert observer.get_state()["role"] == "observer"
    assert "enabled" in observer.set_header_only(True)["message"]
    assert "disabled" in observer.set_header_only(False)["message"]
    # State-gated read commands are gated by state, not by role: in IDLE the
    # limits queries fail with a plain (uncoded) error, not "forbidden".
    if observer.get_state()["state"] == "idle":
        with pytest.raises(CommandError) as exc_info:
            observer.command({"cmd": "get_frame_duration_limits"},
                             expect=("frame_duration_limits",))
        assert exc_info.value.code is None


def test_invalid_stream_options_are_rejected(observer: CherupiClient) -> None:
    for body in (
        {"cmd": "start_stream", "camera_id": 0, "subsample": 3},
        {"cmd": "start_stream", "camera_id": 0, "max_fps": -1},
        {"cmd": "start_stream", "camera_id": -1},
        {"cmd": "start_stream"},
        {"cmd": "start_stream", "camera_id": 99999},
    ):
        with pytest.raises(CommandError) as exc_info:
            observer.command(body)
        assert exc_info.value.code is None, body


# --- state pushes -------------------------------------------------------------


def test_observer_receives_state_pushes(
    client: CherupiClient, observer: CherupiClient, discovered_cameras: list
) -> None:
    observer.drain_messages()
    client.configure(**TEST_CONFIG)
    m = observer.wait_for_state("configured")
    assert m["cause"] == "configure"
    assert m["role"] == "observer"
    assert m["commander_connected"] is True

    client.start_cameras()
    try:
        m = observer.wait_for_state("running")
        assert m["cause"] == "start_cameras"
    finally:
        client.stop_cameras()
    m = observer.wait_for_state("configured")
    assert m["cause"] == "stop_cameras"

    client.unconfigure()
    m = observer.wait_for_state("idle")
    assert m["cause"] == "unconfigure"


def test_commander_also_receives_state_pushes(
    client: CherupiClient, discovered_cameras: list
) -> None:
    client.drain_messages()
    client.configure(**TEST_CONFIG)
    m = client.wait_for_state("configured")
    assert m["cause"] == "configure"
    assert m["role"] == "commander"
    client.unconfigure()
    assert client.wait_for_state("idle")["cause"] == "unconfigure"


def test_observer_is_told_when_a_commander_comes_and_goes(
    server_url: str, observer: CherupiClient
) -> None:
    observer.drain_messages()
    c = CherupiClient(server_url)
    c.connect()
    try:
        m = observer.wait_for_message(
            lambda m: m.get("type") == "state"
            and m.get("cause") == "commander_connected"
        )
        assert m["commander_connected"] is True
    finally:
        c.close()
    m = observer.wait_for_message(
        lambda m: m.get("type") == "state"
        and m.get("cause") == "commander_disconnected"
    )
    assert m["commander_connected"] is False


# --- persistent subscriptions & independence ---------------------------------


def test_observer_subscription_persists_across_camera_restart(
    client: CherupiClient, observer: CherupiClient, discovered_cameras: list
) -> None:
    cam_id = discovered_cameras[0]["id"]

    # Subscribing while IDLE is allowed for an observer.
    resp = observer.start_stream(cam_id)
    assert resp["message"].startswith(f"Started streaming camera {cam_id}")
    assert "subscribed" in resp["message"]

    client.configure(**TEST_CONFIG)
    client.start_cameras()
    try:
        frame = observer.next_frame(timeout=10.0)
        assert frame.camera_id == cam_id
        assert not frame.is_header_only
        assert len(frame.data) == frame.total_size

        client.stop_cameras()
        # Whatever was in flight lands; after that, silence.
        time.sleep(1.0)
        observer.drain_frames()
        with pytest.raises(TimeoutError):
            observer.next_frame(timeout=1.5)

        # The subscription survives the restart: no new start_stream needed.
        client.start_cameras()
        frame = observer.next_frame(timeout=10.0)
        assert frame.camera_id == cam_id
    finally:
        try:
            client.stop_cameras()
        except CommandError:
            pass


def test_observer_stream_is_independent_of_the_commander_stream(
    running_client: CherupiClient, observer: CherupiClient,
    discovered_cameras: list,
) -> None:
    c = running_client
    cam_id = discovered_cameras[0]["id"]

    # Commander not streaming at all: the observer still gets payload.
    observer.start_stream(cam_id)
    frame = observer.next_frame(timeout=10.0)
    assert not frame.is_header_only

    # Commander in header-only mode: no effect on the observer's payload.
    c.set_header_only(True)
    c.start_stream(cam_id)
    _next_frame_matching(c, lambda f: f.is_header_only)
    observer.drain_frames()
    _next_frame_matching(observer, lambda f: not f.is_header_only)

    # Observer in header-only mode: no effect on the commander's payload.
    observer.set_header_only(True)
    c.set_header_only(False)
    _next_frame_matching(c, lambda f: not f.is_header_only)
    _next_frame_matching(observer, lambda f: f.is_header_only)

    c.stop_stream(cam_id)
    observer.set_header_only(False)


def test_observer_keeps_streaming_after_commander_disconnects(
    server_url: str, observer: CherupiClient
) -> None:
    a = CherupiClient(server_url)
    a.connect()
    cams = a.discover()
    if not cams:
        a.close()
        pytest.skip("No cameras discovered — hardware required for e2e")
    cam_id = cams[0]["id"]
    a.configure(**TEST_CONFIG)
    a.start_cameras()
    try:
        observer.start_stream(cam_id, subsample=2)
        observer.next_frame(timeout=10.0)
        # Commander drops (crash / network): cameras keep running and the
        # observer keeps its subscription.
        a.abrupt_close()
        observer.drain_frames()
        frame = observer.next_frame(timeout=10.0)
        assert frame.camera_id == cam_id
    finally:
        b = CherupiClient(server_url)
        _retry_connect(b, timeout=5.0)
        try:
            for step in (b.stop_cameras, b.unconfigure):
                try:
                    step()
                except CommandError:
                    pass
        finally:
            b.close()
        time.sleep(0.5)


# --- stream options -----------------------------------------------------------


def test_observer_subsample_scales_geometry(
    running_client: CherupiClient, observer: CherupiClient,
    discovered_cameras: list,
) -> None:
    c = running_client
    cam_id = discovered_cameras[0]["id"]
    c.start_stream(cam_id)
    full = c.next_frame(timeout=5.0)
    c.stop_stream(cam_id)

    def expected(dim: int, factor: int) -> int:
        return (dim // factor) & ~1

    observer.start_stream(cam_id, subsample=2)
    frame = observer.next_frame(timeout=10.0)
    assert frame.width == expected(full.width, 2)
    assert frame.height == expected(full.height, 2)
    assert frame.bytes_per_line == frame.width  # tightly packed
    assert frame.total_size == frame.width * frame.height * 3 // 2
    assert len(frame.data) == frame.total_size
    assert frame.pixel_format == full.pixel_format

    # Re-issuing start_stream updates the options in place.
    observer.start_stream(cam_id, subsample=4)
    frame = _next_frame_matching(
        observer, lambda f: f.width == expected(full.width, 4)
    )
    assert frame.height == expected(full.height, 4)
    assert len(frame.data) == frame.width * frame.height * 3 // 2

    # Header-only + subsample reports the reduced geometry with no payload.
    observer.set_header_only(True)
    frame = _next_frame_matching(observer, lambda f: f.is_header_only)
    assert frame.width == expected(full.width, 4)
    assert frame.total_size == 0
    observer.set_header_only(False)


def test_observer_max_fps_caps_delivery(
    running_client: CherupiClient, observer: CherupiClient,
    discovered_cameras: list,
) -> None:
    cam_id = discovered_cameras[0]["id"]
    observer.start_stream(cam_id, max_fps=2.0, subsample=4)
    observer.next_frame(timeout=10.0)
    observer.drain_frames()

    window = 3.0
    t0 = time.monotonic()
    received = []
    while time.monotonic() - t0 < window:
        try:
            observer.next_frame(timeout=0.5)
            received.append(time.monotonic() - t0)
        except TimeoutError:
            pass
    # 2 fps over 3 s is 6 frames; allow one extra for edge alignment. The
    # camera runs at >= 10 fps, so an uncapped stream would deliver far more.
    assert 3 <= len(received) <= 7, received
    # And no two deliveries closer than the cap allows (with scheduling slack).
    gaps = [b - a for a, b in zip(received, received[1:])]
    assert all(g >= 0.4 for g in gaps), gaps
