"""Binary CHUN/CHNK protocol integrity checks."""

from __future__ import annotations

from .client import CherupiClient, FOURCC_YU12


def test_frame_header_fields(
    configured_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    c = configured_client
    c.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        c.start_stream(cam_id)
        frame = c.next_frame(timeout=5.0)

        assert frame.pixel_format == FOURCC_YU12
        assert frame.width > 0 and frame.height > 0
        assert frame.bytes_per_line >= frame.width
        assert frame.total_chunks >= 1
        # YUV420: stride × H for Y + stride/2 × H/2 for each of U and V.
        assert frame.total_size == frame.bytes_per_line * frame.height * 3 // 2
        # Reassembled payload length matches header.
        assert len(frame.data) == frame.total_size

        c.stop_stream(cam_id)
    finally:
        c.stop_cameras()


def test_frame_uuid_unique_per_frame(
    configured_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """frame_uuid should be unique across a short run (no reassembly collisions)."""
    c = configured_client
    c.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        c.start_stream(cam_id)
        uuids: list[int] = []
        for _ in range(10):
            frame = c.next_frame(timeout=5.0)
            uuids.append(frame.frame_uuid)
        assert len(set(uuids)) == len(uuids), f"Duplicate frame_uuids: {uuids}"
        c.stop_stream(cam_id)
    finally:
        c.stop_cameras()


def test_chunk_reassembly_sized_correctly(
    configured_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """At 1456x1088 YUV420 (~2.38 MB) expect ~73 chunks of 32 KiB."""
    c = configured_client
    c.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        c.start_stream(cam_id)
        frame = c.next_frame(timeout=5.0)
        expected_chunks = (frame.total_size + 32767) // 32768
        assert frame.total_chunks == expected_chunks, (
            f"total_chunks {frame.total_chunks} != expected {expected_chunks} "
            f"for total_size {frame.total_size}"
        )
        c.stop_stream(cam_id)
    finally:
        c.stop_cameras()


def test_no_protocol_errors_during_stream(
    configured_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """Receive ~20 frames and ensure the receive thread didn't fault."""
    c = configured_client
    c.start_cameras()
    try:
        cam_id = discovered_cameras[0]["id"]
        c.start_stream(cam_id)
        for _ in range(20):
            c.next_frame(timeout=5.0)
        assert c.rx_error is None, f"Receive thread error: {c.rx_error!r}"
        c.stop_stream(cam_id)
    finally:
        c.stop_cameras()
