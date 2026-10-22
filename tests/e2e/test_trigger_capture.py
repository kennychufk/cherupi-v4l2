"""trigger_capture: one frame per armed camera, on demand, sent to the commander.

Requires live hardware. Built for calibration rigs — a robotic arm moves the
camera, comes to a complete stop, and only then asks for a frame — so these
tests focus on what that loop depends on: the ack names exactly one frame per
armed camera, that frame then arrives (`frame_kind = 2`) whatever the stream
subscriptions, nothing is saved on the server, and one capture runs at a time.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from .client import (
    DETECTION_CHECKERBOARD,
    FOURCC_YU12,
    FRAME_KIND_LIVE,
    FRAME_KIND_TRIGGER,
    CherupiClient,
    CommandError,
)
from .conftest import TEST_CONFIG


def _yuv_bytes(frame) -> int:
    # Rows are padded to the stride (2048 for a 2028-wide IMX477 frame).
    return frame.bytes_per_line * frame.height * 3 // 2


def test_trigger_capture_sends_the_frame_and_saves_nothing(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """No subscription needed; the frame is the one the ack names."""
    cam_id = discovered_cameras[0]["id"]
    client.configure(**TEST_CONFIG)
    client.set_process_mode("none", output_dir=str(tmp_output_dir))
    client.start_cameras()

    result = client.trigger_capture(camera_id=cam_id)
    assert result["cancelled"] is False
    assert result["trigger_id"] == 1
    (capture,) = result["captures"]
    assert capture["camera_id"] == cam_id
    assert "filename" not in capture

    frame = client.next_capture_frame(FRAME_KIND_TRIGGER, result["trigger_id"])
    assert frame.camera_id == cam_id
    assert frame.frame_id == capture["frame_id"]
    assert frame.pixel_format == FOURCC_YU12
    assert (frame.width, frame.height) == (TEST_CONFIG["width"], TEST_CONFIG["height"])
    assert len(frame.data) == _yuv_bytes(frame)
    # Not subscribed: no live frame arrived alongside it.
    time.sleep(0.5)
    assert all(f.frame_kind != FRAME_KIND_LIVE for f in client.drain_frames())

    # Ids count up within a session.
    again = client.trigger_capture(camera_id=cam_id)
    assert again["trigger_id"] == 2
    client.next_capture_frame(FRAME_KIND_TRIGGER, 2)

    client.stop_cameras()
    assert not any(tmp_output_dir.iterdir()), "trigger_capture wrote to disk"


def test_trigger_capture_arms_all_running_cameras(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """Without `camera_id`, one trigger captures from every running camera."""
    client.configure(**TEST_CONFIG)
    client.start_cameras()

    result = client.trigger_capture(timeout=15.0)
    assert result["cancelled"] is False
    expected = {c["id"] for c in discovered_cameras}
    assert {c["camera_id"] for c in result["captures"]} == expected
    got = set()
    for _ in expected:
        frame = client.next_capture_frame(FRAME_KIND_TRIGGER, result["trigger_id"])
        got.add(frame.camera_id)
    assert got == expected


def test_trigger_capture_skip_frames(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """`skip_frames` discards settling frames before the one that is kept."""
    cam_id = discovered_cameras[0]["id"]
    client.configure(**TEST_CONFIG)
    client.start_cameras()
    time.sleep(0.5)

    first = client.trigger_capture(camera_id=cam_id)
    client.next_capture_frame(FRAME_KIND_TRIGGER, first["trigger_id"])
    second = client.trigger_capture(camera_id=cam_id, skip_frames=3)
    client.next_capture_frame(FRAME_KIND_TRIGGER, second["trigger_id"])

    # 3 discarded + the kept frame: at least 4 frames after the first.
    assert second["captures"][0]["frame_id"] >= first["captures"][0]["frame_id"] + 4


def test_send_image_is_independent_of_header_only(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    client.configure(**TEST_CONFIG)
    client.start_cameras()
    client.start_stream(cam_id)

    # Live frames carry pixels, the trigger frame only its header.
    result = client.trigger_capture(camera_id=cam_id, send_image=False)
    frame = client.next_capture_frame(FRAME_KIND_TRIGGER, result["trigger_id"])
    assert frame.is_header_only
    assert frame.frame_id == result["captures"][0]["frame_id"]
    assert (frame.width, frame.height) == (TEST_CONFIG["width"], TEST_CONFIG["height"])

    # Live frames header-only, the trigger frame with its pixels.
    client.set_header_only(True)
    result = client.trigger_capture(camera_id=cam_id)
    frame = client.next_capture_frame(FRAME_KIND_TRIGGER, result["trigger_id"])
    assert not frame.is_header_only
    assert len(frame.data) == _yuv_bytes(frame)
    live = [f for f in client.drain_frames() if f.frame_kind == FRAME_KIND_LIVE]
    assert all(f.is_header_only for f in live)
    # ...and the live stream carries on header-only.
    nxt = client.next_frame(timeout=5.0)
    assert nxt.frame_kind == FRAME_KIND_LIVE and nxt.is_header_only


def test_trigger_frame_gets_the_modes_detection(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """In a detector mode the triggered frame is detected (whether or not a
    board is in view, the block's kind says which detector ran)."""
    cam_id = discovered_cameras[0]["id"]
    client.configure(**TEST_CONFIG)
    client.set_process_mode("checkerboard", save_frames=False)
    client.start_cameras()

    result = client.trigger_capture(camera_id=cam_id, send_image=False)
    frame = client.next_capture_frame(FRAME_KIND_TRIGGER, result["trigger_id"],
                                      timeout=30.0)
    assert frame.detection_kind == DETECTION_CHECKERBOARD


def test_one_capture_at_a_time_and_stop_cancels(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    client.configure(**TEST_CONFIG)
    client.start_cameras()
    # 100 skipped frames keep the trigger pending for a few seconds.
    client._send({"cmd": "trigger_capture", "camera_id": cam_id, "skip_frames": 100})
    time.sleep(0.3)
    with pytest.raises(CommandError, match="already in progress"):
        client.trigger_capture(camera_id=cam_id)

    client._send({"cmd": "stop_cameras"})
    msg = client.wait_for_message(lambda m: m.get("type") == "trigger_result",
                                  timeout=15.0)
    assert msg["cancelled"] is True
    assert msg["captures"] == []


def test_observer_never_receives_capture_frames(
    client: CherupiClient, observer: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    observer.start_stream(cam_id)
    client.configure(**TEST_CONFIG)
    client.start_cameras()
    result = client.trigger_capture(camera_id=cam_id)
    client.next_capture_frame(FRAME_KIND_TRIGGER, result["trigger_id"])
    time.sleep(1.0)
    frames = observer.drain_frames()
    assert frames, "the observer's live stream stopped"
    assert all(f.frame_kind == FRAME_KIND_LIVE for f in frames)


def test_trigger_capture_rejects_bad_fields(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    client.configure(**TEST_CONFIG)
    client.start_cameras()
    with pytest.raises(CommandError, match="send_image"):
        client.command({"cmd": "trigger_capture", "send_image": "no"},
                       expect=("trigger_result",))
    with pytest.raises(CommandError, match="camera_id"):
        client.command({"cmd": "trigger_capture", "camera_id": -1},
                       expect=("trigger_result",))


def test_trigger_capture_requires_running_state(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """Triggering before start_cameras is a state-machine error, not a crash."""
    client.configure(**TEST_CONFIG)
    with pytest.raises(CommandError, match="RUNNING"):
        client.trigger_capture()


def test_trigger_and_stack_are_no_longer_modes(client: CherupiClient) -> None:
    for mode in ("trigger", "stack"):
        with pytest.raises(CommandError, match="Invalid process mode"):
            client.set_process_mode(mode)
