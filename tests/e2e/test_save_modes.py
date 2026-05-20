"""Frame-saver modes produce artifacts on disk.

These tests require live hardware and drive the actual FrameSaver. They're
deliberately lightweight — we just check that files appear with the expected
shape; content validation is the job of yuv420_convert / the converter.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from .client import CherupiClient


def _yuv_files(path: Path) -> list[Path]:
    return sorted(path.glob("cam*-*.yuv"))


def test_save_mode_buffer(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """BUFFER: frames accumulate in RAM, flushed on stop_cameras."""
    out = tmp_output_dir / "buffer"
    out.mkdir()
    client.configure(width=1456, height=1088)
    client.set_save_mode("buffer", output_dir=str(out))
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(1.5)
    client.stop_stream(discovered_cameras[0]["id"])
    stop = client.stop_cameras()

    files = _yuv_files(out)
    assert files, f"BUFFER mode produced no files in {out}"
    assert stop.get("frames_saved", 0) >= len(files) - 1  # +/-1 for rounding


def test_save_mode_batch(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """BATCH: multi-threaded writer flushes every batch_size frames."""
    out = tmp_output_dir / "batch"
    out.mkdir()
    client.configure(width=1456, height=1088)
    client.set_save_mode(
        "batch", output_dir=str(out), batch_size=5, writer_threads=2
    )
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    # Need at least `batch_size` frames to trigger a flush.
    time.sleep(2.5)
    client.stop_stream(discovered_cameras[0]["id"])
    stop = client.stop_cameras()

    files = _yuv_files(out)
    assert files, f"BATCH mode produced no files in {out}"
    # Filename pattern: cam{id}-{frame_id}.yuv
    cam_id = discovered_cameras[0]["id"]
    assert any(
        f.name.startswith(f"cam{cam_id}-") for f in files
    ), f"No files matching cam{cam_id}-*.yuv in {[f.name for f in files]}"
    assert stop.get("frames_saved", 0) > 0


def test_save_mode_checkerboard_finds_nothing(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """CHECKERBOARD with no target visible — just asserts the mode runs cleanly.

    We cannot assume a physical board is in front of the camera, so we only
    verify the command is accepted and the run completes without errors.
    If the camera happens to see a board, files will appear; if not, none
    will — either way, the pipeline should exit cleanly.
    """
    out = tmp_output_dir / "checkerboard"
    out.mkdir()
    client.configure(width=1456, height=1088)
    client.set_save_mode(
        "checkerboard",
        output_dir=str(out),
        checkerboard_rows=8,
        checkerboard_cols=11,
    )
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(2.0)
    client.stop_stream(discovered_cameras[0]["id"])
    client.stop_cameras()  # should not raise


def test_save_mode_checkerboard2x2_finds_nothing(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """CHECKERBOARD2X2 with no target visible — just asserts the mode runs cleanly.

    Mirrors the CHECKERBOARD test above. The 2x2 mode splits each frame into 4
    quadrants and detects per-quadrant in parallel; with no board in view, no
    files should land, but the pipeline must still shut down cleanly.
    """
    out = tmp_output_dir / "checkerboard2x2"
    out.mkdir()
    client.configure(width=1456, height=1088)
    client.set_save_mode(
        "checkerboard2x2",
        output_dir=str(out),
        checkerboard_rows=8,
        checkerboard_cols=11,
        checkerboard_num_threads=4,
    )
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(2.0)
    client.stop_stream(discovered_cameras[0]["id"])
    client.stop_cameras()  # should not raise


def test_save_mode_none(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """NONE: no files written regardless of how long we run.

    frames_saved is a session-cumulative counter on the server, so we can't
    assert it equals zero (prior tests in the same session may have saved
    frames). The file-system check below is the canonical invariant for
    NONE mode.
    """
    out = tmp_output_dir / "none"
    out.mkdir()
    client.configure(width=1456, height=1088)
    client.set_save_mode("none", output_dir=str(out))
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(1.0)
    client.stop_stream(discovered_cameras[0]["id"])
    client.stop_cameras()

    assert not _yuv_files(out), f"Expected no files in {out}"


@pytest.mark.parametrize(
    "mode", ["none", "buffer", "batch", "checkerboard", "checkerboard2x2"]
)
def test_set_save_mode_accepted_in_any_state(
    client: CherupiClient, mode: str, tmp_output_dir: Path
) -> None:
    """set_save_mode is allowed in any state (IDLE, CONFIGURED, RUNNING)."""
    out = tmp_output_dir / f"anystate_{mode}"
    out.mkdir()
    client.set_save_mode(mode, output_dir=str(out))  # IDLE
    client.discover()
    client.configure(width=1456, height=1088)
    client.set_save_mode(mode, output_dir=str(out))  # CONFIGURED
    client.start_cameras()
    try:
        client.set_save_mode(mode, output_dir=str(out))  # RUNNING
    finally:
        client.stop_cameras()
