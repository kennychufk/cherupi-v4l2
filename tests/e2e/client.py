"""WebSocket client for the cherupi-v4l2 camera server.

Speaks the v2 CHUN/CHNK binary protocol defined in docs/websocket-protocol.md.
Designed for use from pytest end-to-end tests but has no test-framework
dependency and can be driven from a plain script.

Typical usage:

    with CherupiClient("ws://localhost:9001") as c:
        c.discover()
        c.configure(width=1456, height=1088)
        c.start_cameras()
        c.start_stream(0)
        frame = c.next_frame(timeout=5.0)
        assert frame.pixel_format == FOURCC_YU12
        c.stop_cameras()
"""

from __future__ import annotations

import json
import os
import queue
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# Ensure any ambient http_proxy env var doesn't black-hole localhost upgrades.
# The server runs on the same host as the tests, so we never want a proxy for
# loopback targets. websocket-client's `http_no_proxy` kwarg is only consulted
# when you also pass an explicit proxy host, so set the env var instead.
_existing_no_proxy = os.environ.get("no_proxy", os.environ.get("NO_PROXY", ""))
_loopback_hosts = {"localhost", "127.0.0.1", "::1"}
_no_proxy_entries = {
    h.strip() for h in _existing_no_proxy.split(",") if h.strip()
}
if not _loopback_hosts.issubset(_no_proxy_entries):
    os.environ["no_proxy"] = ",".join(_no_proxy_entries | _loopback_hosts)

import websocket  # pip install websocket-client  # noqa: E402


# --- Wire-format constants (keep in sync with src/types.hpp) ------------------

CHUN_MAGIC = 0x4348554E  # 'CHUN'
CHNK_MAGIC = 0x43484E4B  # 'CHNK'
PROTOCOL_VERSION = 2

# ChunkStartMarker (8 bytes) + ChunkHeader (40 bytes)
START_STRUCT = struct.Struct("<II")  # magic, version
HEADER_STRUCT = struct.Struct("<IIIIIIIIII")  # 10 × uint32
START_PLUS_HEADER_SIZE = START_STRUCT.size + HEADER_STRUCT.size  # 48

# ChunkData (16-byte header)
CHUNK_STRUCT = struct.Struct("<IIII")  # magic, frame_uuid, chunk_index, chunk_size
CHUNK_HEADER_SIZE = CHUNK_STRUCT.size  # 16

MAX_CHUNK_PAYLOAD = 32 * 1024

# FourCC for YUV420 (V4L2_PIX_FMT_YUV420 = 'YU12')
FOURCC_YU12 = 0x32315559


# --- Result types -------------------------------------------------------------


@dataclass
class Frame:
    """A fully reassembled frame."""

    frame_uuid: int
    frame_id: int
    camera_id: int
    total_chunks: int
    total_size: int
    bytes_per_line: int
    width: int
    height: int
    pixel_format: int
    frames_saved: int
    data: bytes  # empty in header-only mode

    @property
    def is_header_only(self) -> bool:
        return self.total_chunks == 0


@dataclass
class _PartialFrame:
    header: Frame
    chunks: Dict[int, bytes] = field(default_factory=dict)

    def is_complete(self) -> bool:
        return len(self.chunks) == self.header.total_chunks

    def assemble(self) -> Frame:
        data = b"".join(self.chunks[i] for i in range(self.header.total_chunks))
        return Frame(**{**self.header.__dict__, "data": data})


class ProtocolError(RuntimeError):
    pass


class CommandError(RuntimeError):
    """Raised when the server returns `{"type":"error", ...}` for a command."""

    def __init__(self, message: str, response: Dict[str, Any]):
        super().__init__(message)
        self.response = response


# --- Client -------------------------------------------------------------------


class CherupiClient:
    """Single-client WebSocket harness for camera_ws_server.

    Command methods block until the server sends a matching `status` / `error`
    (or `discovery` for `discover`). Binary frames arrive asynchronously and
    are exposed through `next_frame()` / `drain_frames()`.
    """

    def __init__(self, url: str, open_timeout: float = 5.0):
        self.url = url
        self._ws: Optional[websocket.WebSocket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._open_timeout = open_timeout

        self._frames: "queue.Queue[Frame]" = queue.Queue()
        self._responses: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self._partials: Dict[int, _PartialFrame] = {}

        self._rx_error: Optional[BaseException] = None

    # -- lifecycle -----------------------------------------------------------

    def connect(self) -> None:
        self._ws = websocket.create_connection(
            self.url, timeout=self._open_timeout, enable_multithread=True
        )
        self._stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="cherupi-client-rx", daemon=True
        )
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
        self._ws = None
        self._rx_thread = None

    def __enter__(self) -> "CherupiClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # noqa: ARG002
        self.close()

    # -- receive loop --------------------------------------------------------

    def _rx_loop(self) -> None:
        assert self._ws is not None
        try:
            while not self._stop.is_set():
                opcode, data = self._ws.recv_data()
                if data is None:
                    break
                if opcode == websocket.ABNF.OPCODE_TEXT:
                    self._handle_text(data.decode("utf-8"))
                elif opcode == websocket.ABNF.OPCODE_BINARY:
                    self._handle_binary(data)
                elif opcode in (websocket.ABNF.OPCODE_CLOSE,):
                    break
                # ping/pong handled inside websocket-client
        except (websocket.WebSocketConnectionClosedException, OSError):
            pass
        except Exception as e:  # noqa: BLE001 — surface on next command
            self._rx_error = e

    def _handle_text(self, text: str) -> None:
        try:
            msg = json.loads(text)
        except json.JSONDecodeError as e:
            raise ProtocolError(f"Server sent non-JSON text: {text!r}") from e
        self._responses.put(msg)

    def _handle_binary(self, data: bytes) -> None:
        if len(data) < 4:
            raise ProtocolError(f"Binary frame too short: {len(data)} bytes")
        (magic,) = struct.unpack_from("<I", data, 0)
        if magic == CHUN_MAGIC:
            self._handle_chunk_start(data)
        elif magic == CHNK_MAGIC:
            self._handle_chunk_data(data)
        else:
            raise ProtocolError(f"Unknown binary magic 0x{magic:08X}")

    def _handle_chunk_start(self, data: bytes) -> None:
        if len(data) != START_PLUS_HEADER_SIZE:
            raise ProtocolError(
                f"CHUN packet wrong size: got {len(data)}, "
                f"expected {START_PLUS_HEADER_SIZE}"
            )
        magic, version = START_STRUCT.unpack_from(data, 0)
        if magic != CHUN_MAGIC:
            raise ProtocolError(f"CHUN magic mismatch: 0x{magic:08X}")
        if version != PROTOCOL_VERSION:
            raise ProtocolError(f"Unsupported version {version}")

        (
            frame_uuid,
            frame_id,
            camera_id,
            total_chunks,
            total_size,
            bytes_per_line,
            width,
            height,
            pixel_format,
            frames_saved,
        ) = HEADER_STRUCT.unpack_from(data, START_STRUCT.size)

        header = Frame(
            frame_uuid=frame_uuid,
            frame_id=frame_id,
            camera_id=camera_id,
            total_chunks=total_chunks,
            total_size=total_size,
            bytes_per_line=bytes_per_line,
            width=width,
            height=height,
            pixel_format=pixel_format,
            frames_saved=frames_saved,
            data=b"",
        )

        if total_chunks == 0:
            # Header-only frame — deliver immediately.
            self._frames.put(header)
        else:
            self._partials[frame_uuid] = _PartialFrame(header=header)

    def _handle_chunk_data(self, data: bytes) -> None:
        magic, frame_uuid, chunk_index, chunk_size = CHUNK_STRUCT.unpack_from(data, 0)
        if magic != CHNK_MAGIC:
            raise ProtocolError(f"CHNK magic mismatch: 0x{magic:08X}")
        payload = data[CHUNK_HEADER_SIZE : CHUNK_HEADER_SIZE + chunk_size]
        if len(payload) != chunk_size:
            raise ProtocolError(
                f"CHNK payload truncated: advertised {chunk_size}, got {len(payload)}"
            )

        partial = self._partials.get(frame_uuid)
        if partial is None:
            # Stale / unmatched chunk — drop silently (server GC after 5s).
            return
        partial.chunks[chunk_index] = payload
        if partial.is_complete():
            del self._partials[frame_uuid]
            self._frames.put(partial.assemble())

    # -- command API ---------------------------------------------------------

    def _send(self, obj: Dict[str, Any]) -> None:
        if self._ws is None:
            raise RuntimeError("Client not connected")
        if self._rx_error is not None:
            raise self._rx_error
        self._ws.send(json.dumps(obj))

    def _expect_response(
        self, expected_types: tuple, timeout: float
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"No {expected_types} response within {timeout}s"
                )
            try:
                msg = self._responses.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError(
                    f"No {expected_types} response within {timeout}s"
                )
            msg_type = msg.get("type")
            if msg_type == "error":
                raise CommandError(msg.get("message", "<no message>"), msg)
            if msg_type in expected_types:
                return msg
            # Ignore unrelated messages (shouldn't happen — server always
            # responds to a command with status/error/discovery).

    def command(
        self,
        cmd: Dict[str, Any],
        expect: tuple = ("status",),
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Send a command and wait for `expect`-type response. Raises on error."""
        self._send(cmd)
        return self._expect_response(expect, timeout)

    # Thin convenience wrappers — intentionally dumb so tests can build
    # unusual command payloads (e.g. missing fields) by calling `command()`
    # directly.

    def discover(self, timeout: float = 5.0) -> List[Dict[str, Any]]:
        resp = self.command({"cmd": "discover"}, expect=("discovery",), timeout=timeout)
        return resp.get("cameras", [])

    def configure(self, **params: Any) -> Dict[str, Any]:
        return self.command({"cmd": "configure", "params": params})

    def unconfigure(self) -> Dict[str, Any]:
        return self.command({"cmd": "unconfigure"})

    def set_save_mode(self, mode: str, **params: Any) -> Dict[str, Any]:
        body: Dict[str, Any] = {"cmd": "set_save_mode", "mode": mode}
        if params:
            body["params"] = params
        return self.command(body)

    def start_cameras(self) -> Dict[str, Any]:
        return self.command({"cmd": "start_cameras"})

    def stop_cameras(self, timeout: float = 15.0) -> Dict[str, Any]:
        return self.command({"cmd": "stop_cameras"}, timeout=timeout)

    def start_stream(self, camera_id: int) -> Dict[str, Any]:
        return self.command({"cmd": "start_stream", "camera_id": camera_id})

    def stop_stream(self, camera_id: int) -> Dict[str, Any]:
        return self.command({"cmd": "stop_stream", "camera_id": camera_id})

    def reset_frame_counts(self) -> Dict[str, Any]:
        return self.command({"cmd": "reset_frame_counts"})

    def set_header_only(self, enabled: bool) -> Dict[str, Any]:
        return self.command({"cmd": "set_header_only", "enabled": enabled})

    # -- frame API -----------------------------------------------------------

    def next_frame(self, timeout: float = 5.0) -> Frame:
        try:
            return self._frames.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f"No frame received within {timeout}s")

    def drain_frames(self) -> List[Frame]:
        out: List[Frame] = []
        while True:
            try:
                out.append(self._frames.get_nowait())
            except queue.Empty:
                return out

    @property
    def rx_error(self) -> Optional[BaseException]:
        return self._rx_error
