"""Protocol-level abuse: malformed inputs, wrong opcodes, pipelining,
out-of-range configure values.

These tests do not exercise the camera state machine — they exercise the
server's robustness when the client violates protocol expectations. The
contract is: respond with `{"type":"error", ...}` and keep the connection
alive. The server must never crash.
"""

from __future__ import annotations

import json

import pytest

from .client import CherupiClient, CommandError


# --- malformed messages -----------------------------------------------------


def test_binary_frame_from_client_returns_error(client: CherupiClient) -> None:
    """Binary opcode is server→client only; client binary must error.

    Verifies the connection survives — we issue a normal command afterwards.
    """
    assert client._ws is not None
    client._ws.send_binary(b"\x00\x01\x02\x03")
    msg = client._responses.get(timeout=2.0)
    assert msg.get("type") == "error"

    # Connection still works.
    cams = client.discover()
    assert isinstance(cams, list)


def test_empty_json_returns_error(client: CherupiClient) -> None:
    """`{}` has no `cmd` field → error."""
    with pytest.raises(CommandError):
        client.command({})


def test_non_string_cmd_returns_error(client: CherupiClient) -> None:
    """`cmd` must be a string."""
    with pytest.raises(CommandError):
        client.command({"cmd": 42})


def test_null_cmd_returns_error(client: CherupiClient) -> None:
    with pytest.raises(CommandError):
        client.command({"cmd": None})


def test_array_payload_returns_error(client: CherupiClient) -> None:
    """A JSON array (not an object) must error without crashing the parser."""
    assert client._ws is not None
    client._ws.send("[1,2,3]")
    msg = client._responses.get(timeout=2.0)
    assert msg.get("type") == "error"
    # Connection still works.
    client.discover()


# --- pipelining -------------------------------------------------------------


def test_pipelined_commands_processed_in_order(client: CherupiClient) -> None:
    """Three back-to-back sends → three responses, in order, types correct.

    Per the contract: rapid-fire pipelined sends without waiting for responses
    are supported. The uWS handler is single-threaded so per-command ordering
    is guaranteed.
    """
    assert client._ws is not None
    client._ws.send(json.dumps({"cmd": "discover"}))
    client._ws.send(json.dumps({"cmd": "reset_frame_counts"}))
    client._ws.send(json.dumps({"cmd": "set_header_only", "enabled": False}))

    r1 = client._responses.get(timeout=3.0)
    r2 = client._responses.get(timeout=3.0)
    r3 = client._responses.get(timeout=3.0)
    assert r1.get("type") == "discovery"
    assert r2.get("type") == "status"
    assert r3.get("type") == "status"


def test_pipelined_mixed_valid_and_invalid(client: CherupiClient) -> None:
    """Valid command followed by invalid command: both responses arrive."""
    assert client._ws is not None
    client._ws.send(json.dumps({"cmd": "discover"}))
    client._ws.send("not json at all")

    r1 = client._responses.get(timeout=3.0)
    r2 = client._responses.get(timeout=3.0)
    assert r1.get("type") == "discovery"
    assert r2.get("type") == "error"
    # Connection still healthy.
    client.discover()


# --- configure out-of-range -------------------------------------------------
#
# Contract: the server should *not* clamp or pre-validate dimensions. It
# passes them to libcamera, which either coerces (libcamera's own
# CameraConfiguration::validate adjusts to a supported mode) or returns an
# error that we surface as a CommandError. Either outcome is acceptable. The
# only thing we forbid is a crash.


def _server_still_alive(c: CherupiClient) -> None:
    """Sentinel: a fresh `discover` must succeed after each abuse case."""
    cams = c.discover()
    assert isinstance(cams, list)


def test_configure_zero_dimensions_does_not_crash(client: CherupiClient) -> None:
    client.discover()
    try:
        client.configure(width=0, height=0)
    except CommandError:
        pass
    else:
        # libcamera accepted (likely after coercion); roll back so the
        # fixture teardown can return to IDLE cleanly.
        try:
            client.unconfigure()
        except CommandError:
            pass
    _server_still_alive(client)


def test_configure_negative_dimensions_does_not_crash(
    client: CherupiClient,
) -> None:
    """Width/height are uint32 in protocol; nlohmann coerces a JSON negative
    number to a wrap-around uint or throws. Either way, no crash."""
    client.discover()
    assert client._ws is not None
    client._ws.send(json.dumps({
        "cmd": "configure",
        "params": {"width": -1, "height": -1},
    }))
    # Whatever the server does with this, *some* response must arrive.
    msg = client._responses.get(timeout=5.0)
    assert msg.get("type") in ("status", "error")
    if msg.get("type") == "status":
        try:
            client.unconfigure()
        except CommandError:
            pass
    _server_still_alive(client)


def test_configure_absurdly_huge_dimensions_does_not_crash(
    client: CherupiClient,
) -> None:
    client.discover()
    try:
        # Larger than any sensor mode; libcamera will either coerce down to
        # the nearest supported size or reject.
        client.configure(width=99999, height=99999)
    except CommandError:
        pass
    else:
        try:
            client.unconfigure()
        except CommandError:
            pass
    _server_still_alive(client)


def test_configure_extra_unknown_fields_ignored(client: CherupiClient) -> None:
    """Unknown fields under `params` must be silently ignored, not rejected."""
    client.discover()
    client.configure(
        width=1456,
        height=1088,
        # extra fields the server doesn't know about
        future_param=True,
        nested={"a": 1, "b": [2, 3]},
    )
    client.unconfigure()
