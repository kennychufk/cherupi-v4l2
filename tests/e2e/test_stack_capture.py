"""Raw frame stacking: stack_capture, its stack_result and the stacked image.

Requires live hardware. A board in view is not required: these tests check
the protocol — settling on the pinned exposure, the ack's shape, the stacked
image that follows it (`frame_kind = 1`), send_image, one capture at a time,
and cancellation.
"""

from __future__ import annotations

import struct
import time

import pytest

from .client import (
    DETECTION_CHECKERBOARD,
    DETECTION_NONE,
    FOURCC_SF32,
    FRAME_KIND_LIVE,
    FRAME_KIND_STACK,
    CherupiClient,
    CommandError,
)
from .conftest import TEST_CONFIG


def _start(client: CherupiClient, mode: str = "none", **params) -> None:
    resp = client.configure(**{**TEST_CONFIG, "raw": True})
    assert resp["raw"] is not None, resp
    assert resp["raw"]["width"] == TEST_CONFIG["width"]
    client.set_process_mode(mode, **params)
    client.start_cameras()


def test_stack_capture_needs_the_raw_stream(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    resp = client.configure(**TEST_CONFIG)
    assert resp["raw"] is None
    client.start_cameras()
    with pytest.raises(CommandError, match="raw"):
        client.stack_capture(num_frames=2)


def test_stack_capture_reports_then_sends_the_image(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    _start(client)
    result = client.stack_capture(camera_id=cam_id, num_frames=4, exposure_time=20000)
    assert result["cancelled"] is False
    assert result["stack_id"] == 1
    (c,) = result["captures"]
    assert "error" not in c, c
    assert c["camera_id"] == cam_id
    assert c["frames_stacked"] == 4
    assert c["last_frame_id"] - c["first_frame_id"] >= 3
    assert abs(c["exposure_time"] - 20000) <= 400
    assert abs(c["analogue_gain"] - 1.0) <= 0.03
    assert c["width"] == TEST_CONFIG["width"] // 2
    assert c["height"] == TEST_CONFIG["height"] // 2
    assert 0.0 <= c["white_fraction"] <= 1.0
    assert c["accumulate_ms"] > 0
    # Detection rides on the frame; nothing is saved.
    assert "sets" not in c and "filename" not in c

    # No subscription: the image is sent because it was asked for.
    frame = client.next_capture_frame(FRAME_KIND_STACK, result["stack_id"])
    assert frame.pixel_format == FOURCC_SF32
    assert frame.camera_id == cam_id
    assert frame.frame_id == c["last_frame_id"]
    assert frame.detection_kind == DETECTION_NONE  # process mode none
    assert (frame.width, frame.height) == (c["width"], c["height"])
    assert frame.bytes_per_line == frame.width * 4
    assert len(frame.data) == frame.width * frame.height * 4
    # The image is linear light above black: nothing far below zero.
    samples = struct.unpack_from(f"<{frame.width}f", frame.data,
                                 (frame.height // 2) * frame.bytes_per_line)
    assert min(samples) > -500.0
    assert max(samples) > 0.0

    # Ids count up within a session.
    again = client.stack_capture(camera_id=cam_id, num_frames=1)
    assert again["stack_id"] == 2
    client.next_capture_frame(FRAME_KIND_STACK, 2)


def test_send_image_false_sends_only_the_header(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """Whatever the header-only state: here live frames keep their pixels."""
    cam_id = discovered_cameras[0]["id"]
    _start(client)
    client.start_stream(cam_id, subsample=4)
    result = client.stack_capture(camera_id=cam_id, num_frames=2, send_image=False)
    (c,) = result["captures"]
    frame = client.next_capture_frame(FRAME_KIND_STACK, result["stack_id"])
    assert frame.is_header_only
    # Full stack geometry, not the subscription's subsampled size.
    assert (frame.width, frame.height) == (c["width"], c["height"])
    # The live stream keeps its pixels.
    live = client.next_frame(timeout=5.0)
    assert live.frame_kind == FRAME_KIND_LIVE and not live.is_header_only


def test_stack_gets_the_modes_detection(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    _start(client, "checkerboard", save_frames=False)
    result = client.stack_capture(camera_id=cam_id, num_frames=2, send_image=False)
    frame = client.next_capture_frame(FRAME_KIND_STACK, result["stack_id"],
                                      timeout=30.0)
    assert frame.detection_kind == DETECTION_CHECKERBOARD


def test_one_capture_at_a_time_and_stop_cancels(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    _start(client)
    client._send({"cmd": "stack_capture", "camera_id": cam_id, "num_frames": 1000})
    time.sleep(0.3)
    # Either kind of capture is refused while one is outstanding.
    with pytest.raises(CommandError, match="already in progress"):
        client.stack_capture(camera_id=cam_id, num_frames=1)
    with pytest.raises(CommandError, match="already in progress"):
        client.trigger_capture(camera_id=cam_id)

    client._send({"cmd": "stop_cameras"})
    msg = client.wait_for_message(lambda m: m.get("type") == "stack_result",
                                  timeout=15.0)
    assert msg["cancelled"] is True
    assert msg["captures"][0]["error"] == "cancelled"
    time.sleep(0.5)
    assert all(f.frame_kind != FRAME_KIND_STACK for f in client.drain_frames())
