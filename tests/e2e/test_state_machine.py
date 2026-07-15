"""End-to-end state-machine gate checks.

Each test starts from IDLE (conftest teardown drives the server back), so
tests can assert specific rejected-state transitions without worrying about
test-ordering state leakage.
"""

from __future__ import annotations

import time
from typing import Callable

import pytest

from .client import CherupiClient, CommandError, Frame


# --- IDLE-state rejections --------------------------------------------------


def test_start_cameras_requires_configured(client: CherupiClient) -> None:
    """start_cameras in IDLE must fail."""
    with pytest.raises(CommandError):
        client.start_cameras()


def test_start_stream_requires_running(client: CherupiClient) -> None:
    """start_stream in CONFIGURED must fail."""
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras")
    client.configure(width=1456, height=1088)
    with pytest.raises(CommandError):
        client.start_stream(cams[0]["id"])


def test_stop_cameras_requires_running(client: CherupiClient) -> None:
    """stop_cameras in IDLE and CONFIGURED must fail."""
    with pytest.raises(CommandError):
        client.stop_cameras()

    client.discover()
    client.configure(width=1456, height=1088)
    with pytest.raises(CommandError):
        client.stop_cameras()


# --- Always-allowed commands ------------------------------------------------


def test_reset_frame_counts_allowed_any_state(client: CherupiClient) -> None:
    """reset_frame_counts must succeed in IDLE, CONFIGURED, and RUNNING."""
    # IDLE
    client.reset_frame_counts()
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras")
    client.configure(width=1456, height=1088)
    # CONFIGURED
    client.reset_frame_counts()
    client.start_cameras()
    try:
        # RUNNING
        client.reset_frame_counts()
    finally:
        client.stop_cameras()


# --- Error-response checks (state-independent) ------------------------------


def test_unknown_command_returns_error(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": "not_a_real_command"})


def test_malformed_json_returns_error(client: CherupiClient) -> None:
    """Non-JSON text on the command channel must produce an error response."""
    assert client._ws is not None
    client._ws.send("this is not json")
    msg = client._responses.get(timeout=2.0)
    assert msg.get("type") == "error"


def test_invalid_save_mode_returns_error(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.set_process_mode("not-a-mode")


def test_unknown_camera_id_returns_error(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """start_stream with a bogus camera id must error (tested in RUNNING)."""
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        bad_id = max(c["id"] for c in discovered_cameras) + 999
        with pytest.raises(CommandError):
            client.start_stream(bad_id)
    finally:
        client.stop_cameras()


# --- stop_stream gating -----------------------------------------------------


def test_stop_stream_in_idle_rejected(client: CherupiClient) -> None:
    """stop_stream is RUNNING-only; from IDLE it must error."""
    with pytest.raises(CommandError):
        client.stop_stream(0)


def test_stop_stream_in_configured_rejected(client: CherupiClient) -> None:
    """stop_stream is RUNNING-only; from CONFIGURED it must error."""
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras")
    client.configure(width=1456, height=1088)
    with pytest.raises(CommandError):
        client.stop_stream(cams[0]["id"])


def test_stop_stream_unknown_camera_returns_error(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """stop_stream of a camera that wasn't streaming must error."""
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        # Never started — must error.
        with pytest.raises(CommandError):
            client.stop_stream(cam_id)
    finally:
        client.stop_cameras()


# --- start_stream idempotency ----------------------------------------------


def test_start_stream_idempotent_for_same_camera(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """start_stream twice for the same camera silently re-succeeds.

    This is the documented contract: a re-issued start_stream is a no-op
    with a status response. After two calls, exactly one stop_stream is
    needed to halt delivery (no error on the first stop, error on the
    second).
    """
    cam_id = discovered_cameras[0]["id"]
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        client.start_stream(cam_id)
        # Second call must not error.
        client.start_stream(cam_id)
        # Frames still flow.
        frame = client.next_frame(timeout=5.0)
        assert frame.camera_id == cam_id
        # Single stop_stream halts the (single) logical stream.
        client.stop_stream(cam_id)
        with pytest.raises(CommandError):
            client.stop_stream(cam_id)
    finally:
        client.stop_cameras()


# --- mid-stream control commands -------------------------------------------


def _next_frame_matching(
    c: CherupiClient,
    pred: Callable[[Frame], bool],
    *,
    timeout: float = 5.0,
    max_frames: int = 30,
) -> Frame:
    """Read frames until one matches `pred` or we exhaust budget.

    Useful right after a mode-switch command — frames already in flight may
    still reflect the old mode, so we skip over them.
    """
    deadline = time.monotonic() + timeout
    seen = 0
    while seen < max_frames:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        f = c.next_frame(timeout=remaining)
        seen += 1
        if pred(f):
            return f
    raise AssertionError(
        f"No frame matched predicate within {timeout}s / {max_frames} frames"
    )


def test_set_header_only_toggle_mid_stream(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """Toggling header-only mid-stream switches the binary frame format.

    With header-only enabled the server must emit `total_chunks=0` headers
    with no `CHNK` packets; toggling back must resume full data frames.
    """
    cam_id = discovered_cameras[0]["id"]
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        client.start_stream(cam_id)
        # Baseline: data frames.
        first = _next_frame_matching(
            client, lambda f: f.camera_id == cam_id and not f.is_header_only
        )
        assert len(first.data) == first.total_size

        # Switch to header-only.
        client.set_header_only(True)
        client.drain_frames()  # discard any in-flight pre-toggle frames
        ho = _next_frame_matching(client, lambda f: f.is_header_only)
        assert ho.total_chunks == 0
        assert ho.total_size == 0
        assert ho.data == b""

        # Switch back to full frames.
        client.set_header_only(False)
        client.drain_frames()
        back = _next_frame_matching(client, lambda f: not f.is_header_only)
        assert len(back.data) == back.total_size
    finally:
        client.stop_cameras()


def test_reset_frame_counts_mid_stream_does_not_disrupt(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """reset_frame_counts mid-stream rolls frame_id back without a stall."""
    cam_id = discovered_cameras[0]["id"]
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        client.start_stream(cam_id)
        # Advance the counter.
        last: Frame | None = None
        for _ in range(8):
            last = client.next_frame(timeout=5.0)
        assert last is not None
        pre_reset_id = last.frame_id
        assert pre_reset_id >= 1, "expected frame_id to advance"

        client.reset_frame_counts()
        client.drain_frames()  # discard in-flight pre-reset frames

        # Next freshly-captured frame should have a much smaller frame_id.
        post = _next_frame_matching(
            client,
            lambda f: f.camera_id == cam_id and f.frame_id < pre_reset_id,
            timeout=5.0,
            max_frames=30,
        )
        assert post.frame_id < pre_reset_id
    finally:
        client.stop_cameras()


def test_discover_allowed_during_running(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """`discover` is documented as allowed in any state."""
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        cams = client.discover()
        assert len(cams) == len(discovered_cameras)
    finally:
        client.stop_cameras()


# --- many-cycle reliability -------------------------------------------------


def test_many_configure_unconfigure_cycles(client: CherupiClient) -> None:
    """Repeated IDLE↔CONFIGURED cycles are reliable (no libcamera leaks)."""
    client.discover()
    cycles = 5
    for i in range(cycles):
        client.configure(width=1456, height=1088)
        # Even cycles also visit RUNNING to exercise the full pipeline.
        if i % 2 == 0:
            client.start_cameras()
            client.stop_cameras()
        client.unconfigure()
