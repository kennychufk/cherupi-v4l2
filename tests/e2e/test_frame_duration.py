"""End-to-end tests for `set_frame_duration` and `get_frame_duration_limits`.

Verifies state-machine gating (IDLE rejected, CONFIGURED + RUNNING accepted),
input validation, and that frames keep flowing across frame-duration toggles
mid-stream. The actual framerate effect is verified out-of-band; here we only
assert that the protocol contract holds and the streaming pipeline survives
runtime changes.
"""

from __future__ import annotations

import pytest

from .client import CherupiClient, CommandError


def test_set_frame_duration_rejected_in_idle(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": "set_frame_duration", "frame_duration": 33333})


def test_get_frame_duration_limits_rejected_in_idle(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command(
            {"cmd": "get_frame_duration_limits"},
            expect=("frame_duration_limits",),
        )


def test_set_frame_duration_in_configured(configured_client: CherupiClient) -> None:
    # Lock at ~30 fps.
    resp = configured_client.command(
        {"cmd": "set_frame_duration", "frame_duration": 33333}
    )
    assert "locked" in resp.get("message", "").lower()
    # Unset (any non-positive value).
    resp = configured_client.command(
        {"cmd": "set_frame_duration", "frame_duration": -1}
    )
    assert "unset" in resp.get("message", "").lower()


def test_get_frame_duration_limits_in_configured(
    configured_client: CherupiClient,
) -> None:
    # Initially unset → `current` is null.
    resp = configured_client.command(
        {"cmd": "get_frame_duration_limits"},
        expect=("frame_duration_limits",),
    )
    assert resp["type"] == "frame_duration_limits"
    assert isinstance(resp["min"], int) and resp["min"] > 0
    assert isinstance(resp["max"], int) and resp["max"] > resp["min"]
    assert resp["current"] is None

    # After locking, `current` reflects {min, max} with min == max.
    configured_client.command(
        {"cmd": "set_frame_duration", "frame_duration": 33333}
    )
    resp = configured_client.command(
        {"cmd": "get_frame_duration_limits"},
        expect=("frame_duration_limits",),
    )
    assert resp["current"] == {"min": 33333, "max": 33333}

    # Back to unset.
    configured_client.command(
        {"cmd": "set_frame_duration", "frame_duration": 0}
    )
    resp = configured_client.command(
        {"cmd": "get_frame_duration_limits"},
        expect=("frame_duration_limits",),
    )
    assert resp["current"] is None


def test_set_frame_duration_running_keeps_streaming(
    running_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    running_client.start_stream(cam_id)
    try:
        # Toggle through several frame durations; each should yield a fresh
        # frame within the timeout, proving the streaming path was not broken
        # by attaching ControlList to recycled requests.
        for fd in (16667, 33333, -1, 50000, -1):
            running_client.command(
                {"cmd": "set_frame_duration", "frame_duration": fd}
            )
            f = running_client.next_frame(timeout=5.0)
            assert f.camera_id == cam_id
    finally:
        running_client.stop_stream(cam_id)


def test_set_frame_duration_rejects_missing_field(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command({"cmd": "set_frame_duration"})


def test_set_frame_duration_rejects_non_numeric(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_frame_duration", "frame_duration": "slow"}
        )


def test_set_frame_duration_rejects_above_cap(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_frame_duration", "frame_duration": 1_000_000_001}
        )
