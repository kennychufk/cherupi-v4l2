"""Happy-path lifecycle: discover → configure → start → stream → stop.

Each test starts from a clean IDLE state (the `client` fixture's teardown
walks the server back through stop_cameras/unconfigure as needed).
"""

from __future__ import annotations

import pytest

from .client import CherupiClient, CommandError, FOURCC_YU12


def test_full_lifecycle(client: CherupiClient) -> None:
    cameras = client.discover()
    assert len(cameras) >= 1, "Expected at least one camera on hardware test rig"
    for cam in cameras:
        assert "id" in cam and "type" in cam

    # libcamera may adjust the requested size to the nearest supported
    # mode, so don't pin exact width/height — check structural invariants.
    client.configure(width=1456, height=1088)
    client.start_cameras()

    cam_id = cameras[0]["id"]
    client.start_stream(cam_id)
    frame = client.next_frame(timeout=5.0)
    assert frame.camera_id == cam_id
    assert frame.width > 0 and frame.height > 0
    assert frame.bytes_per_line >= frame.width
    assert frame.pixel_format == FOURCC_YU12
    # YUV420 payload = Y plane (stride × H) + U + V planes (each stride/2 × H/2).
    assert frame.total_size == frame.bytes_per_line * frame.height * 3 // 2
    assert len(frame.data) == frame.total_size

    client.stop_stream(cam_id)
    stop_resp = client.stop_cameras()
    assert "frames_saved" in stop_resp
    assert "bytes_written" in stop_resp


def test_multi_camera_roundrobin(client: CherupiClient) -> None:
    """With two cameras streaming, we eventually see one frame per camera."""
    cameras = client.discover()
    if len(cameras) < 2:
        pytest.skip(f"Test requires 2 cameras, got {len(cameras)}")

    client.configure(width=1456, height=1088)
    client.start_cameras()
    for cam in cameras:
        client.start_stream(cam["id"])

    seen_ids: set[int] = set()
    for _ in range(30):
        frame = client.next_frame(timeout=5.0)
        seen_ids.add(frame.camera_id)
        if len(seen_ids) == len(cameras):
            break
    assert seen_ids == {cam["id"] for cam in cameras}

    client.stop_cameras()


def test_reconfigure_requires_unconfigure(client: CherupiClient) -> None:
    """Demonstrates the full IDLE→CONFIGURED→IDLE cycle via unconfigure.

    `configure` is IDLE-only; to re-apply a different config without
    disconnecting, the client must `unconfigure` first.
    """
    client.discover()
    client.configure(width=1456, height=1088)

    # Second configure in CONFIGURED state must error.
    with pytest.raises(CommandError):
        client.configure(width=1456, height=1088)

    # unconfigure returns to IDLE; configure now succeeds again.
    client.unconfigure()
    client.configure(width=1456, height=1088)

    # And in RUNNING it must also error.
    client.start_cameras()
    with pytest.raises(CommandError):
        client.configure(width=1456, height=1088)
    client.stop_cameras()


def test_unconfigure_requires_configured(client: CherupiClient) -> None:
    """unconfigure is rejected from IDLE and from RUNNING."""
    # IDLE — no configure yet.
    with pytest.raises(CommandError):
        client.unconfigure()

    # RUNNING
    client.discover()
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        with pytest.raises(CommandError):
            client.unconfigure()
    finally:
        client.stop_cameras()
