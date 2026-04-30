"""End-to-end tests for the `set_lens_position` command.

Verifies state-machine gating (IDLE rejected, CONFIGURED + RUNNING accepted),
input validation, and that frames keep flowing across focus toggles mid-stream.
The actual lens motion is verified out-of-band via `v4l2-ctl --get-ctrl
focus_absolute` per the manual verification steps in the plan; here we only
assert that the protocol contract holds and the streaming pipeline survives
focus changes.
"""

from __future__ import annotations

import pytest

from .client import CherupiClient, CommandError


def test_set_lens_position_rejected_in_idle(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": "set_lens_position", "lens_position": 4.0})


def test_set_lens_position_in_configured(configured_client: CherupiClient) -> None:
    # Manual focus at 4.0 dioptres.
    resp = configured_client.command(
        {"cmd": "set_lens_position", "lens_position": 4.0}
    )
    assert "manual" in resp.get("message", "").lower()
    # AF sentinel (any negative value).
    resp = configured_client.command(
        {"cmd": "set_lens_position", "lens_position": -1}
    )
    assert "autofocus" in resp.get("message", "").lower()


def test_set_lens_position_running_keeps_streaming(
    running_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    running_client.start_stream(cam_id)
    try:
        # Toggle through several focus settings; each should yield a fresh
        # frame within the timeout, proving the streaming path was not
        # broken by attaching ControlList to recycled requests.
        for lp in (0.0, 5.0, -1, 8.0, -1):
            running_client.command(
                {"cmd": "set_lens_position", "lens_position": lp}
            )
            f = running_client.next_frame(timeout=5.0)
            assert f.camera_id == cam_id
    finally:
        running_client.stop_stream(cam_id)


def test_set_lens_position_rejects_missing_field(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command({"cmd": "set_lens_position"})


def test_set_lens_position_rejects_non_numeric(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_lens_position", "lens_position": "near"}
        )


def test_set_lens_position_rejects_out_of_range(
    configured_client: CherupiClient,
) -> None:
    with pytest.raises(CommandError):
        configured_client.command(
            {"cmd": "set_lens_position", "lens_position": 1e6}
        )
