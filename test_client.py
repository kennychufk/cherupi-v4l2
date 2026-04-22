#!/usr/bin/env python3
"""Simple test client for camera_ws_server.

Usage:
    python3 test_client.py [--host HOST] [--port PORT]
"""

import argparse
import json
import struct
import sys
import time
import websocket  # pip install websocket-client

FRAME_HEADER_FORMAT = "<IIIII"  # frame_id, camera_id, bytes_per_line, width, height
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FORMAT)

frames_received = 0


def on_message(ws, message):
    global frames_received
    if isinstance(message, bytes):
        if len(message) < FRAME_HEADER_SIZE:
            print(f"[WARN] Binary message too short ({len(message)} bytes)")
            return
        frame_id, camera_id, bpl, width, height = struct.unpack_from(
            FRAME_HEADER_FORMAT, message
        )
        payload = len(message) - FRAME_HEADER_SIZE
        frames_received += 1
        print(
            f"[FRAME] cam={camera_id} frame={frame_id} "
            f"{width}x{height} stride={bpl} payload={payload}B "
            f"(total received: {frames_received})"
        )
    else:
        try:
            msg = json.loads(message)
            print(f"[{msg.get('type','msg').upper()}] {msg}")
        except json.JSONDecodeError:
            print(f"[RAW] {message}")


def on_error(ws, error):
    print(f"[ERROR] {error}", file=sys.stderr)


def on_close(ws, code, reason):
    print(f"[CLOSE] code={code} reason={reason}")


def on_open(ws):
    print("[OPEN] Connected to server")

    def send(cmd):
        print(f"[SEND] {cmd}")
        ws.send(json.dumps(cmd))
        time.sleep(0.5)

    send({"cmd": "discover"})
    send({
        "cmd": "configure",
        "params": {"width": 2328, "height": 1748},
    })
    send({"cmd": "start_cameras"})
    send({"cmd": "start_stream", "camera_id": 0})

    # Receive frames for 3 seconds then stop
    time.sleep(3)
    send({"cmd": "stop_stream", "camera_id": 0})
    send({"cmd": "stop_cameras"})
    time.sleep(0.5)
    ws.close()


def main():
    parser = argparse.ArgumentParser(description="camera_ws_server test client")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9001)
    args = parser.parse_args()

    url = f"ws://{args.host}:{args.port}"
    print(f"Connecting to {url}")

    ws = websocket.WebSocketApp(
        url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )
    ws.run_forever()
    print(f"\nDone. Total frames received: {frames_received}")


if __name__ == "__main__":
    main()
