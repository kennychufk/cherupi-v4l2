"""Single-client enforcement: second WebSocket upgrade returns HTTP 503."""

from __future__ import annotations

import pytest
import websocket

from .client import CherupiClient


def test_second_connection_rejected(server_url: str, client: CherupiClient) -> None:
    """Client is already connected via the `client` fixture.

    Opening a second WebSocket must fail at the HTTP upgrade with 503.
    """
    with pytest.raises(websocket.WebSocketException) as excinfo:
        websocket.create_connection(server_url, timeout=2.0)
    # websocket-client surfaces the status code in the exception message.
    # We don't pin the exact class/string but we do expect 503 somewhere.
    assert "503" in str(excinfo.value), (
        f"Expected 503 in rejection, got: {excinfo.value!r}"
    )
