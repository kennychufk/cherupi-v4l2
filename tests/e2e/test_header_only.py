"""Header-only mode: total_chunks=0, no CHNK packets follow."""

from __future__ import annotations

from .client import CherupiClient


def test_header_only_no_payload(
    configured_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    c = configured_client
    c.set_header_only(True)
    c.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        c.start_stream(cam_id)
        # Drop the first frame — a race at toggle time could still queue
        # a pre-toggle frame through the pipeline. Then validate the next 5.
        _ = c.next_frame(timeout=5.0)
        for _ in range(5):
            frame = c.next_frame(timeout=5.0)
            assert frame.is_header_only, (
                f"Expected header-only, got total_chunks={frame.total_chunks}"
            )
            assert frame.total_size == 0
            assert len(frame.data) == 0
            # Metadata still populated.
            assert frame.width > 0 and frame.height > 0
        c.stop_stream(cam_id)
    finally:
        c.stop_cameras()


def test_header_only_toggle_restores_payload(
    configured_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    c = configured_client
    c.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        c.start_stream(cam_id)

        # Normal mode — frames should have payload.
        frame = c.next_frame(timeout=5.0)
        assert not frame.is_header_only

        # Enable header-only and drain a few frames to clear in-flight state.
        c.set_header_only(True)
        for _ in range(3):
            c.next_frame(timeout=5.0)
        # Now the next frame should be header-only.
        frame = c.next_frame(timeout=5.0)
        assert frame.is_header_only

        # Disable — payload should come back.
        c.set_header_only(False)
        for _ in range(3):
            c.next_frame(timeout=5.0)
        frame = c.next_frame(timeout=5.0)
        assert not frame.is_header_only

        c.stop_stream(cam_id)
    finally:
        c.stop_cameras()
