# IMX296 Camera WebSocket Server

A high-performance WebSocket server for streaming raw frames from multiple IMX296 cameras on Raspberry Pi 5.

## Features

- Auto-discovery of all connected IMX296 cameras
- Simultaneous control of multiple cameras
- Real-time streaming of raw SRGGB10P frames over WebSocket
- Adaptive frame skipping based on network congestion
- Optional frame saving to SSD (buffer or batch mode)
- Single WebSocket connection with multiplexed camera streams
- Zero-copy frame transmission where possible

## Dependencies

- CMake 3.14 or higher
- C++17 compiler
- OpenSSL development libraries
- zlib development libraries
- Linux V4L2 and Media Controller APIs

## Building

### Install System Dependencies

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libssl-dev \
    zlib1g-dev
```

### Build the Server

```bash
mkdir build
cd build
cmake ..
make -j4
```

CMake will automatically download and build the following dependencies:
- uWebSockets (with uSockets)
- nlohmann/json

## Usage

Run the server:

```bash
sudo ./camera_ws_server
```

The server listens on port 9001 by default.

## WebSocket Protocol

### Client to Server Commands

All commands are sent as JSON text messages:

```json
// Discover cameras
{"cmd": "discover"}

// Configure all cameras (must be done before starting)
{"cmd": "configure", "params": {
    "width": 1456,
    "height": 1088,
    "crop_width": 1456,
    "crop_height": 1088,
    "crop_left": 0,
    "crop_top": 0
}}

// Set frame saving mode
{"cmd": "set_save_mode", "mode": "none|buffer|batch", "params": {
    "prefix": "camera",
    "batch_size": 10,
    "writer_threads": 4
}}

// Start all cameras
{"cmd": "start_cameras"}

// Start streaming from a specific camera
{"cmd": "start_stream", "camera_id": 0}

// Stop streaming from a specific camera
{"cmd": "stop_stream", "camera_id": 0}

// Stop all cameras
{"cmd": "stop_cameras"}
```

### Server to Client Messages

Text messages (JSON):

```json
// Discovery response
{"type": "discovery", "cameras": [{"id": 0, "type": "IMX296"}, ...]}

// Status message
{"type": "status", "message": "..."}

// Error message
{"type": "error", "message": "..."}
```

Binary messages (for frames):
```
[FrameHeader (20 bytes)][Raw frame data (SRGGB10P)]

FrameHeader structure:
- frame_id (uint32_t): Frame number (0-indexed)
- camera_id (uint32_t): Camera ID (0-indexed)
- bytes_per_line (uint32_t): Stride of the frame
- width (uint32_t): Frame width in pixels
- height (uint32_t): Frame height in pixels
```

## Example Client (Python)

```python
import asyncio
import websockets
import json
import struct

async def client():
    uri = "ws://localhost:9001"
    
    async with websockets.connect(uri) as websocket:
        # Discover cameras
        await websocket.send(json.dumps({"cmd": "discover"}))
        response = json.loads(await websocket.recv())
        print(f"Found cameras: {response}")
        
        # Configure cameras
        await websocket.send(json.dumps({
            "cmd": "configure",
            "params": {"width": 1456, "height": 1088}
        }))
        await websocket.recv()
        
        # Set save mode
        await websocket.send(json.dumps({
            "cmd": "set_save_mode",
            "mode": "none"
        }))
        await websocket.recv()
        
        # Start cameras
        await websocket.send(json.dumps({"cmd": "start_cameras"}))
        await websocket.recv()
        
        # Start streaming from camera 0
        await websocket.send(json.dumps({
            "cmd": "start_stream",
            "camera_id": 0
        }))
        await websocket.recv()
        
        # Receive frames
        frame_count = 0
        while frame_count < 100:
            message = await websocket.recv()
            
            if isinstance(message, bytes):
                # Parse frame header
                header = struct.unpack("<IIIII", message[:20])
                frame_id, camera_id, bytes_per_line, width, height = header
                
                # Frame data starts after header
                frame_data = message[20:]
                
                print(f"Frame {frame_id} from camera {camera_id}: "
                      f"{width}x{height}, {len(frame_data)} bytes")
                
                frame_count += 1
        
        # Stop streaming
        await websocket.send(json.dumps({
            "cmd": "stop_stream",
            "camera_id": 0
        }))
        
        # Stop cameras
        await websocket.send(json.dumps({"cmd": "stop_cameras"}))

asyncio.run(client())
```

## Performance Notes

- The server uses adaptive frame skipping to maintain real-time streaming
- Only one frame is kept in the network pipeline at a time
- Multiple cameras are handled in round-robin fashion
- Frame saving runs independently of streaming
- Use O_DIRECT for optimal SSD write performance in batch mode

## Limitations

- Only one client connection is allowed at a time
- All cameras use identical configuration
- Cameras can only be configured once per connection
- Frame saving options must be set before starting cameras
