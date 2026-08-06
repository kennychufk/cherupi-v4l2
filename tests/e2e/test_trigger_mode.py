"""TRIGGER process mode: frames hit disk only when the client asks for one.

Requires live hardware. The mode exists for automated calibration rigs — a
robotic arm moves the camera, comes to a complete stop, and only then asks the
server to keep a frame — so these tests focus on the two properties that rig
depends on: nothing is written until a trigger arrives, and each trigger yields
exactly one frame per armed camera plus an ack naming it.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from .client import CherupiClient, CommandError


def _yuv_files(path: Path) -> list[Path]:
    return sorted(path.glob("cam*-*.yuv"))


def test_trigger_mode_saves_only_on_request(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """Free-running frames are discarded; a trigger saves exactly one per camera."""
    out = tmp_output_dir / "trigger"
    out.mkdir()
    cam_id = discovered_cameras[0]["id"]

    client.configure(width=1456, height=1088)
    client.set_process_mode("trigger", output_dir=str(out))
    client.start_cameras()
    client.start_stream(cam_id)

    # Plenty of frames flow past — none of them should be written.
    time.sleep(1.5)
    assert not _yuv_files(out), "trigger mode wrote frames without a trigger"

    result = client.trigger_capture(camera_id=cam_id)
    assert result["cancelled"] is False
    captures = result["captures"]
    assert len(captures) == 1
    assert captures[0]["camera_id"] == cam_id
    assert captures[0]["filename"]

    # The ack fires once the frame is queued for writing; give the writer pool
    # a moment to land it on disk.
    deadline = time.monotonic() + 5.0
    saved = Path(captures[0]["filename"])
    while not saved.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert saved.exists(), f"triggered frame never written: {saved}"
    assert saved.name == f"cam{cam_id}-{captures[0]['frame_id']}.yuv"

    # A second trigger adds exactly one more file.
    client.trigger_capture(camera_id=cam_id)
    deadline = time.monotonic() + 5.0
    while len(_yuv_files(out)) < 2 and time.monotonic() < deadline:
        time.sleep(0.05)

    client.stop_stream(cam_id)
    client.stop_cameras()
    assert len(_yuv_files(out)) == 2


def test_trigger_capture_arms_all_running_cameras(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """Without `camera_id`, one trigger captures from every running camera."""
    out = tmp_output_dir / "trigger_all"
    out.mkdir()

    client.configure(width=1456, height=1088)
    client.set_process_mode("trigger", output_dir=str(out))
    client.start_cameras()
    time.sleep(0.5)

    result = client.trigger_capture(timeout=15.0)
    assert result["cancelled"] is False
    captured_ids = {c["camera_id"] for c in result["captures"]}
    assert captured_ids == {c["id"] for c in discovered_cameras}

    client.stop_cameras()
    assert len(_yuv_files(out)) == len(discovered_cameras)


def test_trigger_capture_skip_frames(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """`skip_frames` discards settling frames before the one that is kept."""
    out = tmp_output_dir / "trigger_skip"
    out.mkdir()
    cam_id = discovered_cameras[0]["id"]

    client.configure(width=1456, height=1088)
    client.set_process_mode("trigger", output_dir=str(out))
    client.start_cameras()
    client.start_stream(cam_id)
    time.sleep(0.5)

    first = client.trigger_capture(camera_id=cam_id)
    second = client.trigger_capture(camera_id=cam_id, skip_frames=3)

    # The skipped trigger must land at least 4 frames later than the immediate
    # one (3 discarded + the kept frame).
    assert second["captures"][0]["frame_id"] >= first["captures"][0]["frame_id"] + 4
    assert second["trigger_id"] > first["trigger_id"]

    client.stop_stream(cam_id)
    client.stop_cameras()


def test_trigger_capture_rejected_outside_trigger_mode(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """The request is refused (and saves nothing) in any other process mode."""
    out = tmp_output_dir / "trigger_wrong_mode"
    out.mkdir()

    client.configure(width=1456, height=1088)
    client.set_process_mode("none", output_dir=str(out))
    client.start_cameras()

    with pytest.raises(CommandError, match="trigger"):
        client.trigger_capture(camera_id=discovered_cameras[0]["id"])

    client.stop_cameras()
    assert not _yuv_files(out)


def test_trigger_capture_requires_running_state(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """Triggering before start_cameras is a state-machine error, not a crash."""
    assert discovered_cameras  # fixture also skips the test without hardware
    out = tmp_output_dir / "trigger_not_running"
    out.mkdir()

    client.configure(width=1456, height=1088)
    client.set_process_mode("trigger", output_dir=str(out))

    with pytest.raises(CommandError, match="RUNNING"):
        client.trigger_capture()
