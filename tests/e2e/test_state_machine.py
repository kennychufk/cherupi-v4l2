"""End-to-end state-machine gate checks.

Each test starts from IDLE (conftest teardown drives the server back), so
tests can assert specific rejected-state transitions without worrying about
test-ordering state leakage.
"""

from __future__ import annotations

import pytest

from .client import CherupiClient, CommandError


# --- IDLE-state rejections --------------------------------------------------


def test_start_cameras_requires_configured(client: CherupiClient) -> None:
    """start_cameras in IDLE must fail."""
    with pytest.raises(CommandError):
        client.start_cameras()


def test_start_stream_requires_running(client: CherupiClient) -> None:
    """start_stream in CONFIGURED must fail."""
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras")
    client.configure(width=1456, height=1088)
    with pytest.raises(CommandError):
        client.start_stream(cams[0]["id"])


def test_stop_cameras_requires_running(client: CherupiClient) -> None:
    """stop_cameras in IDLE and CONFIGURED must fail."""
    with pytest.raises(CommandError):
        client.stop_cameras()

    client.discover()
    client.configure(width=1456, height=1088)
    with pytest.raises(CommandError):
        client.stop_cameras()


# --- Always-allowed commands ------------------------------------------------


def test_reset_frame_counts_allowed_any_state(client: CherupiClient) -> None:
    """reset_frame_counts must succeed in IDLE, CONFIGURED, and RUNNING."""
    # IDLE
    client.reset_frame_counts()
    cams = client.discover()
    if not cams:
        pytest.skip("No cameras")
    client.configure(width=1456, height=1088)
    # CONFIGURED
    client.reset_frame_counts()
    client.start_cameras()
    try:
        # RUNNING
        client.reset_frame_counts()
    finally:
        client.stop_cameras()


# --- Error-response checks (state-independent) ------------------------------


def test_unknown_command_returns_error(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": "not_a_real_command"})


def test_malformed_json_returns_error(client: CherupiClient) -> None:
    """Non-JSON text on the command channel must produce an error response."""
    assert client._ws is not None
    client._ws.send("this is not json")
    msg = client._responses.get(timeout=2.0)
    assert msg.get("type") == "error"


def test_invalid_save_mode_returns_error(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.set_save_mode("not-a-mode")


def test_unknown_camera_id_returns_error(
    client: CherupiClient, discovered_cameras: list[dict]
) -> None:
    """start_stream with a bogus camera id must error (tested in RUNNING)."""
    client.configure(width=1456, height=1088)
    client.start_cameras()
    try:
        bad_id = max(c["id"] for c in discovered_cameras) + 999
        with pytest.raises(CommandError):
            client.start_stream(bad_id)
    finally:
        client.stop_cameras()
