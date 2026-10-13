"""End-to-end tests for the `set_lens_position` command and FocusFoM reporting.

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
from .conftest import TEST_CONFIG


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


# --- FocusFoM (ChunkHeader v7) and its region --------------------------------


def _assert_region_shape(region: dict, width: int, height: int) -> None:
    assert region["frame_width"] == width
    assert region["frame_height"] == height
    # The PiSP CDAF grid is 8x8 and the IPA averages cells 2-5 on both axes,
    # so the block is the middle half of the frame (within a cell's even
    # truncation) and lies wholly inside it.
    assert region["grid"] == {"size": 8, "first_col": 2, "first_row": 2,
                              "num_cols": 4, "num_rows": 4}
    assert 0 < region["x"] and region["x"] + region["width"] < width
    assert 0 < region["y"] and region["y"] + region["height"] < height
    assert abs(region["width"] - width / 2) <= 8
    assert abs(region["height"] - height / 2) <= 8


def test_configure_reports_focus_fom_region(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    resp = client.configure(**TEST_CONFIG)
    _assert_region_shape(resp["focus_fom_region"], TEST_CONFIG["width"],
                         TEST_CONFIG["height"])


def test_get_focus_fom_region_rejected_in_idle(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": "get_focus_fom_region"},
                       expect=("focus_fom_region",))


def test_observer_reads_the_same_focus_fom_region(
    configured_client: CherupiClient, observer: CherupiClient
) -> None:
    resp = observer.command({"cmd": "get_focus_fom_region"},
                            expect=("focus_fom_region",))
    _assert_region_shape(resp, TEST_CONFIG["width"], TEST_CONFIG["height"])
    commander = configured_client.command({"cmd": "get_focus_fom_region"},
                                          expect=("focus_fom_region",))
    assert commander == resp


def test_frames_carry_focus_fom(
    running_client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    cam_id = discovered_cameras[0]["id"]
    running_client.set_header_only(True)
    running_client.start_stream(cam_id)
    try:
        # The IPA reports FocusFoM on every frame once its statistics are
        # flowing; allow a few frames for the first ones to arrive.
        foms = [running_client.next_frame(timeout=5.0).focus_fom
                for _ in range(10)]
        assert all(f >= -1 for f in foms)
        assert any(f >= 0 for f in foms), foms
    finally:
        running_client.stop_stream(cam_id)
        running_client.set_header_only(False)
