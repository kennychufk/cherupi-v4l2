"""Raw frame stacking: stack_capture, its stack_result, and the `stack` mode.

Requires live hardware. A board in view is not required: these tests check
the protocol — settling on the pinned exposure, the result's shape, the
stacked image streamed in `stack` mode, one stack at a time, cancellation,
and saving — with detection off.
"""

from __future__ import annotations

import json
import struct
import time
from pathlib import Path

import pytest

from .client import (
    FOURCC_SF32,
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


def test_stack_capture_pins_exposure_and_reports_the_stack(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    _start(client)
    result = client.stack_capture(
        camera_id=cam_id, num_frames=4, exposure_time=20000, detect="none"
    )
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
    assert c["detect"] == "none" and c["sets"] == []
    assert c["accumulate_ms"] > 0

    # Ids count up within a session.
    again = client.stack_capture(camera_id=cam_id, num_frames=1, detect="none")
    assert again["stack_id"] == 2


def test_stack_mode_streams_only_stacked_images(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    _start(client, "stack", stack_detect="none")
    client.start_stream(cam_id)

    # No live frames in this mode.
    time.sleep(1.0)
    assert client.drain_frames() == []

    result = client.stack_capture(camera_id=cam_id, num_frames=2)
    (c,) = result["captures"]
    assert "error" not in c, c
    frame = client.next_frame(timeout=15.0)
    assert frame.frame_kind == FRAME_KIND_STACK
    assert frame.pixel_format == FOURCC_SF32
    assert frame.frame_id == result["stack_id"]
    assert frame.camera_id == cam_id
    assert (frame.width, frame.height) == (c["width"], c["height"])
    assert frame.bytes_per_line == frame.width * 4
    assert len(frame.data) == frame.width * frame.height * 4
    # The image is linear light above black: nothing far below zero.
    samples = struct.unpack_from(f"<{frame.width}f", frame.data,
                                 (frame.height // 2) * frame.bytes_per_line)
    assert min(samples) > -500.0
    assert max(samples) > 0.0

    # Header-only: the stacked frame's header still arrives, without pixels.
    client.set_header_only(True)
    client.stack_capture(camera_id=cam_id, num_frames=1)
    hdr = client.next_frame(timeout=15.0)
    assert hdr.is_header_only and hdr.frame_kind == FRAME_KIND_STACK


def test_one_stack_at_a_time_and_stop_cancels(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    _start(client)
    client._send({"cmd": "stack_capture", "camera_id": cam_id,
                  "num_frames": 1000, "detect": "none"})
    time.sleep(0.3)
    with pytest.raises(CommandError, match="already pending"):
        client.stack_capture(camera_id=cam_id, num_frames=1)

    client._send({"cmd": "stop_cameras"})
    msg = client.wait_for_message(lambda m: m.get("type") == "stack_result",
                                  timeout=15.0)
    assert msg["cancelled"] is True
    assert msg["captures"][0]["error"] == "cancelled"


def test_save_image_writes_npy_and_sidecar(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    cam_id = discovered_cameras[0]["id"]
    out = tmp_output_dir / "stacks"
    _start(client)
    result = client.stack_capture(camera_id=cam_id, num_frames=2, detect="none",
                                  save_image=True, output_dir=str(out))
    (c,) = result["captures"]
    npy = Path(c["filename"])
    assert npy == out / f"stack{result['stack_id']}-cam{cam_id}.npy"
    data = npy.read_bytes()
    assert data[:6] == b"\x93NUMPY"
    header_len = struct.unpack_from("<H", data, 8)[0]
    header = data[10:10 + header_len].decode()
    assert f"'shape': ({c['height']}, {c['width']})" in header
    assert len(data) == 10 + header_len + c["width"] * c["height"] * 4
    sidecar = json.loads(npy.with_suffix(".json").read_text())
    assert sidecar["frames_stacked"] == 2
