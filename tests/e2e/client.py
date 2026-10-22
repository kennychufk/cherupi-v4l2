"""WebSocket client for the cherupi-v4l2 camera server.

Speaks the v8 CHUN/CHNK binary protocol defined in docs/websocket-protocol.md.
Designed for use from pytest end-to-end tests but has no test-framework
dependency and can be driven from a plain script.

A client connects in one of two **roles** (see protocol §1):

* ``commander`` (default) — read-write; owns the camera lifecycle. Connects to
  the bare URL (path ``/``).
* ``observer`` — read-only; may only subscribe to frames. Connects to
  ``<url>/observer``.

Typical usage:

    with CherupiClient("ws://localhost:9001") as c:
        c.discover()
        c.configure(width=1456, height=1088)
        c.start_cameras()
        c.start_stream(0)
        frame = c.next_frame(timeout=5.0)
        assert frame.pixel_format == FOURCC_YU12
        c.stop_cameras()

    with CherupiClient("ws://localhost:9001", role="observer") as o:
        o.start_stream(0)          # allowed in any state; persistent
        frame = o.next_frame(timeout=30.0)
"""

from __future__ import annotations

import json
import math
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
PROTOCOL_VERSION = 8

# ChunkStartMarker (8 bytes) + ChunkHeader (72 bytes)
START_STRUCT = struct.Struct("<II")  # magic, version
# frame_uuid, frame_id, camera_id, total_chunks, total_size, bytes_per_line,
# width, height, pixel_format, frames_saved, timestamp_us (u64),
# frame_duration_us, corner_block_size, num_corner_sets (u16), capture_id (u16),
# lens_position (f32), af_state (u8), detection_kind (u8), frame_kind (u8),
# reserved2 (u8), focus_fom (i32)
HEADER_STRUCT = struct.Struct("<IIIIIIIIIIQIIHHfBBBBi")
START_PLUS_HEADER_SIZE = START_STRUCT.size + HEADER_STRUCT.size  # 80

# Detection block records (§5.4)
CORNER_SET_HEADER = struct.Struct("<BBH")  # set_id, flags, num_corners
MARKER_SET_HEADER = struct.Struct("<iBBH")  # marker_id, quadrant, flags, num_corners
CORNER_STRUCT = struct.Struct("<ff")

DETECTION_NONE = 0
DETECTION_CHECKERBOARD = 1
DETECTION_ARUCO = 2

# ChunkData (16-byte header)
CHUNK_STRUCT = struct.Struct("<IIII")  # magic, frame_uuid, chunk_index, chunk_size
CHUNK_HEADER_SIZE = CHUNK_STRUCT.size  # 16

MAX_CHUNK_PAYLOAD = 32 * 1024

# FourCC for YUV420 (V4L2_PIX_FMT_YUV420 = 'YU12')
FOURCC_YU12 = 0x32315559
# FourCC of a stacked raw image ('SF32': float32, half resolution, §5.5)
FOURCC_SF32 = 0x32334653

# ChunkHeader.frame_kind
FRAME_KIND_LIVE = 0
FRAME_KIND_STACK = 1  # a stack_capture's image
FRAME_KIND_TRIGGER = 2  # a trigger_capture's frame

ROLE_COMMANDER = "commander"
ROLE_OBSERVER = "observer"

# Sensor model substring sent with `discover` when a call does not name one.
# The server's own default is "imx519"; the conftest overrides this from
# `--sensor` / `CHERUPI_SENSOR` so the suite can run against another module.
DEFAULT_SENSOR: Optional[str] = None


def role_url(base_url: str, role: str) -> str:
    """Return the URL a client of `role` connects to (protocol §1)."""
    base = base_url.rstrip("/")
    if role == ROLE_OBSERVER:
        return base + "/observer"
    if role == ROLE_COMMANDER:
        return base + "/"
    raise ValueError(f"Unknown role {role!r}")


# --- Result types -------------------------------------------------------------


@dataclass
class CornerSet:
    """One record of the detection block: a checkerboard (marker_id is None) or
    an ArUco marker (marker_id set)."""

    set_id: int  # quadrant for 2x2 modes, else 0
    flags: int
    corners: List[tuple]  # [(x, y), ...] in the streamed frame's pixel space
    marker_id: Optional[int] = None


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
    timestamp_us: int = 0
    frame_duration_us: int = 0
    lens_position: float = math.nan
    af_state: int = 0xFF
    detection_kind: int = DETECTION_NONE
    focus_fom: int = -1  # libcamera FocusFoM; -1 if unavailable
    frame_kind: int = FRAME_KIND_LIVE
    # trigger_id / stack_id (low 16 bits) of the capture; 0 on a live frame.
    capture_id: int = 0
    corner_sets: List[CornerSet] = field(default_factory=list)

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

    @property
    def code(self) -> Optional[str]:
        return self.response.get("code")


# --- Client -------------------------------------------------------------------


class CherupiClient:
    """WebSocket harness for camera_ws_server, in either client role.

    Command methods block until the server sends a matching `status` / `error`
    (or `discovery` for `discover`, `state` for `get_state`). Binary frames
    arrive asynchronously and are exposed through `next_frame()` /
    `drain_frames()`. Unsolicited text messages (state pushes, capture_timeout
    errors, trigger_result) are queued too; `next_message()` /
    `wait_for_message()` read them.
    """

    def __init__(self, url: str, open_timeout: float = 5.0,
                 role: str = ROLE_COMMANDER):
        self.role = role
        self.base_url = url
        self.url = role_url(url, role)
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
            target=self._rx_loop, name=f"cherupi-{self.role}-rx", daemon=True
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

    def abrupt_close(self) -> None:
        """Drop the underlying TCP socket without sending a WebSocket close frame.

        Server observes WS close code 1006 (abnormal closure). Used by tests that
        simulate a client crash / network drop mid-session.
        """
        self._stop.set()
        if self._ws is not None:
            sock = getattr(self._ws, "sock", None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
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
        if len(data) < START_PLUS_HEADER_SIZE:
            raise ProtocolError(
                f"CHUN packet too short: got {len(data)}, "
                f"expected at least {START_PLUS_HEADER_SIZE}"
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
            timestamp_us,
            frame_duration_us,
            corner_block_size,
            num_corner_sets,
            capture_id,
            lens_position,
            af_state,
            detection_kind,
            frame_kind,
            _reserved2,
            focus_fom,
        ) = HEADER_STRUCT.unpack_from(data, START_STRUCT.size)

        if len(data) != START_PLUS_HEADER_SIZE + corner_block_size:
            raise ProtocolError(
                f"CHUN packet wrong size: got {len(data)}, expected "
                f"{START_PLUS_HEADER_SIZE} + corner_block_size {corner_block_size}"
            )
        corner_sets = self._parse_detection_block(
            data, START_PLUS_HEADER_SIZE, corner_block_size, num_corner_sets,
            detection_kind,
        )

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
            timestamp_us=timestamp_us,
            frame_duration_us=frame_duration_us,
            lens_position=lens_position,
            af_state=af_state,
            detection_kind=detection_kind,
            focus_fom=focus_fom,
            frame_kind=frame_kind,
            capture_id=capture_id,
            corner_sets=corner_sets,
        )

        if total_chunks == 0:
            # Header-only frame — deliver immediately.
            self._frames.put(header)
        else:
            self._partials[frame_uuid] = _PartialFrame(header=header)

    @staticmethod
    def _parse_detection_block(
        data: bytes, offset: int, size: int, count: int, kind: int
    ) -> List[CornerSet]:
        end = offset + size
        sets: List[CornerSet] = []
        pos = offset
        for _ in range(count):
            if kind == DETECTION_ARUCO:
                if pos + MARKER_SET_HEADER.size > end:
                    raise ProtocolError("MarkerSetHeader overruns detection block")
                marker_id, quadrant, flags, num_corners = MARKER_SET_HEADER.unpack_from(data, pos)
                pos += MARKER_SET_HEADER.size
                set_id = quadrant
            else:
                if pos + CORNER_SET_HEADER.size > end:
                    raise ProtocolError("CornerSetHeader overruns detection block")
                set_id, flags, num_corners = CORNER_SET_HEADER.unpack_from(data, pos)
                pos += CORNER_SET_HEADER.size
                marker_id = None
            if pos + num_corners * CORNER_STRUCT.size > end:
                raise ProtocolError("corner list overruns detection block")
            corners = [
                CORNER_STRUCT.unpack_from(data, pos + i * CORNER_STRUCT.size)
                for i in range(num_corners)
            ]
            pos += num_corners * CORNER_STRUCT.size
            sets.append(CornerSet(set_id=set_id, flags=flags, corners=corners,
                                  marker_id=marker_id))
        if pos != end:
            raise ProtocolError(
                f"detection block has {end - pos} trailing byte(s)"
            )
        return sets

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
                # Unsolicited errors (capture_timeout) are not replies; skip
                # them here and let wait_for_message() pick them up.
                if msg.get("code") == "capture_timeout":
                    continue
                raise CommandError(msg.get("message", "<no message>"), msg)
            if msg_type in expected_types:
                # A pushed `state` is not the reply to get_state.
                if msg_type == "state" and msg.get("cause") not in (None, "query"):
                    continue
                return msg
            # Ignore unrelated messages (state pushes, trigger_result, ...).

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

    def discover(self, sensor: Optional[str] = None,
                 timeout: float = 5.0) -> List[Dict[str, Any]]:
        body: Dict[str, Any] = {"cmd": "discover"}
        sensor = sensor if sensor is not None else DEFAULT_SENSOR
        if sensor:
            body["params"] = {"sensor": sensor}
        resp = self.command(body, expect=("discovery",), timeout=timeout)
        return resp.get("cameras", [])

    def get_state(self, timeout: float = 5.0) -> Dict[str, Any]:
        """Return the full `state` reply (state, role, *_connected, cause)."""
        return self.command({"cmd": "get_state"}, expect=("state",), timeout=timeout)

    def configure(self, **params: Any) -> Dict[str, Any]:
        return self.command({"cmd": "configure", "params": params})

    def unconfigure(self) -> Dict[str, Any]:
        return self.command({"cmd": "unconfigure"})

    def set_process_mode(self, mode: str, **params: Any) -> Dict[str, Any]:
        body: Dict[str, Any] = {"cmd": "set_process_mode", "mode": mode}
        if params:
            body["params"] = params
        return self.command(body)

    def trigger_capture(
        self,
        camera_id: Optional[int] = None,
        skip_frames: Optional[int] = None,
        send_image: Optional[bool] = None,
        timeout: float = 10.0,
    ) -> Dict[str, Any]:
        """Fire a `trigger_capture` and wait for the `trigger_result` ack.

        The ack is asynchronous: it arrives only once every armed camera has
        delivered its triggered frame, so the timeout must cover at least one
        frame interval per camera (plus `skip_frames` more). The frames follow
        as binary `frame_kind = 2` frames (see `next_capture_frame`).
        """
        body: Dict[str, Any] = {"cmd": "trigger_capture"}
        if camera_id is not None:
            body["camera_id"] = camera_id
        if skip_frames is not None:
            body["skip_frames"] = skip_frames
        if send_image is not None:
            body["send_image"] = send_image
        return self.command(body, expect=("trigger_result",), timeout=timeout)

    def stack_capture(self, timeout: float = 30.0, **fields: Any) -> Dict[str, Any]:
        """Run a `stack_capture` and wait for its asynchronous `stack_result`.

        `fields` are the request's (camera_id, num_frames, exposure_time, ...).
        The timeout must cover settling plus num_frames frame intervals.
        """
        body: Dict[str, Any] = {"cmd": "stack_capture", **fields}
        return self.command(body, expect=("stack_result",), timeout=timeout)

    def start_cameras(self) -> Dict[str, Any]:
        return self.command({"cmd": "start_cameras"})

    def stop_cameras(self, timeout: float = 15.0) -> Dict[str, Any]:
        return self.command({"cmd": "stop_cameras"}, timeout=timeout)

    def start_stream(
        self,
        camera_id: int,
        max_fps: Optional[float] = None,
        subsample: Optional[int] = None,
    ) -> Dict[str, Any]:
        body: Dict[str, Any] = {"cmd": "start_stream", "camera_id": camera_id}
        if max_fps is not None:
            body["max_fps"] = max_fps
        if subsample is not None:
            body["subsample"] = subsample
        return self.command(body)

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

    def next_capture_frame(self, frame_kind: int, capture_id: int,
                           timeout: float = 15.0) -> Frame:
        """Pop frames until the one a capture (`frame_kind` STACK or TRIGGER,
        `capture_id` its stack_id / trigger_id) produced; live frames on the
        way are dropped."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"No capture frame (kind {frame_kind}, id {capture_id}) "
                    f"within {timeout}s"
                )
            frame = self.next_frame(timeout=remaining)
            if frame.frame_kind == frame_kind and frame.capture_id == capture_id:
                return frame

    def drain_frames(self) -> List[Frame]:
        out: List[Frame] = []
        while True:
            try:
                out.append(self._frames.get_nowait())
            except queue.Empty:
                return out

    # -- unsolicited message API ---------------------------------------------

    def next_message(self, timeout: float = 5.0) -> Dict[str, Any]:
        """Pop the next queued text message of any type."""
        try:
            return self._responses.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f"No message received within {timeout}s")

    def wait_for_message(self, predicate, timeout: float = 5.0) -> Dict[str, Any]:
        """Pop messages until `predicate(msg)` is true; drops the others."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"No matching message within {timeout}s")
            msg = self.next_message(timeout=remaining)
            if predicate(msg):
                return msg

    def wait_for_state(self, state: str, timeout: float = 5.0) -> Dict[str, Any]:
        """Wait for a pushed `state` message reporting `state`."""
        return self.wait_for_message(
            lambda m: m.get("type") == "state" and m.get("state") == state,
            timeout=timeout,
        )

    def drain_messages(self) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        while True:
            try:
                out.append(self._responses.get_nowait())
            except queue.Empty:
                return out

    @property
    def rx_error(self) -> Optional[BaseException]:
        return self._rx_error
