"""Frame process modes (save side) produce artifacts on disk.

These tests require live hardware and drive the actual FrameProcessor. They're
deliberately lightweight — we just check that files appear with the expected
shape; content validation is the job of yuv420_convert / the converter.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from .client import CherupiClient
from .conftest import TEST_CONFIG


def _yuv_files(path: Path) -> list[Path]:
    return sorted(path.glob("cam*-*.yuv"))


# start_cameras refuses a mode that provably cannot keep up (§4.6): `buffer`
# when the backlog budget cannot hold two seconds of capture, `batch` when the
# measured write rate of output_dir cannot absorb rate x frame size. At a fast
# sensor mode on a 1 GB Pi writing to an SD card both refusals are legitimate,
# so the tests that exercise those modes are not about the guard: they lock
# the frame rate down first (10 fps fits any budget the server would derive),
# and batch additionally starts with allow_overcommit so an SD card measured a
# little under 20 MB/s cannot flip the outcome — the writer's own backlog
# absorbs a few seconds either way and stop_cameras drains it.
TAME_FRAME_DURATION_US = 100_000  # 10 fps
BATCH_GUARD_PARAMS = {"allow_overcommit": True}


def _lock_frame_rate(client: CherupiClient) -> None:
    client.command({"cmd": "set_frame_duration",
                    "frame_duration": TAME_FRAME_DURATION_US})


def test_save_mode_buffer(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """BUFFER: frames accumulate in RAM, flushed on stop_cameras."""
    out = tmp_output_dir / "buffer"
    out.mkdir()
    client.configure(**TEST_CONFIG)
    _lock_frame_rate(client)
    client.set_process_mode("buffer", output_dir=str(out))
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
    client.configure(**TEST_CONFIG)
    _lock_frame_rate(client)
    client.set_process_mode(
        "batch", output_dir=str(out), batch_size=5, writer_threads=2,
        **BATCH_GUARD_PARAMS,
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
    client.configure(**TEST_CONFIG)
    client.set_process_mode(
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
    client.configure(**TEST_CONFIG)
    client.set_process_mode(
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


def test_save_mode_aruco_finds_nothing(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """ARUCO with no marker visible — just asserts the mode runs cleanly.

    Mirrors the CHECKERBOARD test. Dictionary is hard-coded to
    DICT_APRILTAG_16h5. We cannot assume a physical marker is in front of the
    camera, so we only verify the command is accepted (including the aruco-
    specific params) and the run completes without errors.
    """
    out = tmp_output_dir / "aruco"
    out.mkdir()
    client.configure(**TEST_CONFIG)
    client.set_process_mode(
        "aruco",
        output_dir=str(out),
        aruco_full_res_detection=False,
        aruco_corner_refine=False,
    )
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(2.0)
    client.stop_stream(discovered_cameras[0]["id"])
    client.stop_cameras()  # should not raise


def test_save_mode_aruco2x2_finds_nothing(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """ARUCO2X2 with no marker visible — just asserts the mode runs cleanly.

    Mirrors the ARUCO test above; splits each frame into 4 quadrants and
    detects per-quadrant in parallel (batched by aruco_num_threads).
    """
    out = tmp_output_dir / "aruco2x2"
    out.mkdir()
    client.configure(**TEST_CONFIG)
    client.set_process_mode(
        "aruco2x2",
        output_dir=str(out),
        aruco_full_res_detection=False,
        aruco_num_threads=4,
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
    client.configure(**TEST_CONFIG)
    client.set_process_mode("none", output_dir=str(out))
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(1.0)
    client.stop_stream(discovered_cameras[0]["id"])
    client.stop_cameras()

    assert not _yuv_files(out), f"Expected no files in {out}"


@pytest.mark.parametrize(
    "mode",
    [
        "none",
        "buffer",
        "batch",
        "checkerboard",
        "checkerboard2x2",
        "aruco",
        "aruco2x2",
    ],
)
def test_set_process_mode_accepted_in_any_state(
    client: CherupiClient, mode: str, tmp_output_dir: Path
) -> None:
    """set_process_mode is allowed in any state (IDLE, CONFIGURED, RUNNING)."""
    out = tmp_output_dir / f"anystate_{mode}"
    out.mkdir()
    params = {"output_dir": str(out), **BATCH_GUARD_PARAMS}
    client.set_process_mode(mode, **params)  # IDLE
    client.discover()
    client.configure(**TEST_CONFIG)
    _lock_frame_rate(client)  # so `buffer` / `batch` pass the start guard
    client.set_process_mode(mode, **params)  # CONFIGURED
    client.start_cameras()
    try:
        client.set_process_mode(mode, **params)  # RUNNING
    finally:
        client.stop_cameras()


def test_process_mode_save_frames_false_writes_nothing(
    client: CherupiClient,
    discovered_cameras: list[dict],
    tmp_output_dir: Path,
) -> None:
    """save_frames=false runs the detector but persists nothing.

    A detector mode with save_frames disabled must never write a file and must
    report frames_saved == 0, independent of whether any marker is actually in
    view (streaming of detector side-products is unaffected — see the unit
    tests for the streamed-corners guarantee)."""
    out = tmp_output_dir / "aruco_nosave"
    out.mkdir()
    client.configure(**TEST_CONFIG)
    client.set_process_mode("aruco", output_dir=str(out), save_frames=False)
    client.start_cameras()
    client.start_stream(discovered_cameras[0]["id"])
    time.sleep(1.5)
    client.stop_stream(discovered_cameras[0]["id"])
    stop = client.stop_cameras()

    assert not _yuv_files(out), f"save_frames=false still wrote files in {out}"
    assert stop.get("frames_saved", 0) == 0
