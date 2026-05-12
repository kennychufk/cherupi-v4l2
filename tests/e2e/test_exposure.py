"""End-to-end tests for the `set_exposure_time` command.

Verifies state-machine gating (IDLE rejected, CONFIGURED + RUNNING accepted),
input validation, and that frames keep flowing across exposure toggles
mid-stream. The actual shutter effect is verified out-of-band; here we only
assert that the protocol contract holds and the streaming pipeline survives
exposure changes.
"""

from __future__ import annotations

import pytest

from .client import CherupiClient, CommandError


def test_set_exposure_time_rejected_in_idle(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": "set_exposure_time", "exposure_time": 10000})


def test_set_exposure_time_in_configured(configured_client: CherupiClient) -> None:
    # Manual exposure at 10 ms.
    resp = configured_client.command(
        {"cmd": "set_exposure_time", "exposure_time": 10000}
    )
    assert "manual" in resp.get("message", "").lower()
    # Auto AE sentinel (any negative value).
    resp = configured_client.command(
        {"cmd": "set_exposure_time", "exposure_time": -1}
    )
    assert "auto" in resp.get("message", "").lower()


def test_set_exposure_time_running_keeps_streaming(
    running_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    running_client.start_stream(cam_id)
    try:
        # Toggle through several exposure settings; each should yield a fresh
        # frame within the timeout, proving the streaming path was not broken
        # by attaching ControlList to recycled requests.
        for et in (5000, 20000, -1, 10000, -1):
            running_client.command(
                {"cmd": "set_exposure_time", "exposure_time": et}
            )
            f = running_client.next_frame(timeout=5.0)
            assert f.camera_id == cam_id
    finally:
        running_client.stop_stream(cam_id)


def test_set_exposure_time_rejects_missing_field(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command({"cmd": "set_exposure_time"})


def test_set_exposure_time_rejects_non_numeric(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_exposure_time", "exposure_time": "fast"}
        )


def test_set_exposure_time_rejects_out_of_range(
    configured_client: CherupiClient,
) -> None:
    # 0 µs is not a physical shutter duration.
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_exposure_time", "exposure_time": 0}
        )
    # Beyond the server's 1 s cap.
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_exposure_time", "exposure_time": 1_000_001}
        )
