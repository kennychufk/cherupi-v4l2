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
- **Automatic chunked transfer for large frames**

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
    zlib1g-dev \
    libopencv-dev
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

# Checkerboard Save Mode Addition

Add this section to the existing README.md under the "Set frame saving mode" command documentation:

## Frame Saving Modes

### Checkerboard Mode

The `checkerboard` mode automatically detects and saves only frames containing a checkerboard pattern. This mode:

- Debayers each frame using the optimized BayerConverter
- Detects checkerboard patterns using OpenCV
- Saves only the original raw Bayer data of frames containing checkerboards
- Preserves the original frame IDs in filenames

Example command:

```json
// Set checkerboard detection mode with custom parameters
{"cmd": "set_save_mode", "mode": "checkerboard", "params": {
    "prefix": "calib",
    "writer_threads": 4,
    "checkerboard_rows": 8,        // Number of inner corners vertically
    "checkerboard_cols": 11,       // Number of inner corners horizontally
    "checkerboard_full_res_detection": false,  // Use half-res for faster detection
    "checkerboard_num_threads": 4   // Threads for debayering
}}
```

#### Checkerboard Mode Parameters

- `checkerboard_rows` (default: 8): Number of inner corners in the checkerboard pattern vertically
- `checkerboard_cols` (default: 11): Number of inner corners in the checkerboard pattern horizontally
- `checkerboard_full_res_detection` (default: false): Whether to use full resolution for detection (slower but more accurate)
- `checkerboard_num_threads` (default: 4): Number of threads for the debayering process

#### Performance Notes for Checkerboard Mode

- Processing is done sequentially in the capture thread to ensure no frames are missed
- Half-resolution detection (default) provides a good balance between speed and accuracy
- The debayering process uses NEON optimizations for ARM processors
- Original raw Bayer data is saved, not the debayered grayscale image
- Frame IDs are preserved - if frames 0, 2, and 5 contain checkerboards, they will be saved as `prefix_cam0_frame000000.raw`, `prefix_cam0_frame000002.raw`, and `prefix_cam0_frame000005.raw`


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

The server automatically chooses between single-message and chunked transfer based on frame size.

#### Single Frame Message (for frames ≤ 256KB)
```
[FrameHeader (20 bytes)][Raw frame data (SRGGB10P)]

FrameHeader structure:
- frame_id (uint32_t): Frame number (0-indexed)
- camera_id (uint32_t): Camera ID (0-indexed)
- bytes_per_line (uint32_t): Stride of the frame
- width (uint32_t): Frame width in pixels
- height (uint32_t): Frame height in pixels
```

#### Chunked Transfer (for frames > 256KB)

For large frames, the server splits the data into multiple chunks:

1. **Chunk Start Message**:
```
[ChunkStartMarker (8 bytes)][ChunkHeader (28 bytes)]

ChunkStartMarker:
- magic (uint32_t): 0x4348554E ('CHUN')
- version (uint32_t): 1

ChunkHeader:
- frame_id (uint32_t): Frame number
- camera_id (uint32_t): Camera ID
- total_chunks (uint32_t): Total number of chunks
- total_size (uint32_t): Total frame data size in bytes
- bytes_per_line (uint32_t): Stride of the frame
- width (uint32_t): Frame width in pixels
- height (uint32_t): Frame height in pixels
```

2. **Chunk Data Messages** (sent sequentially):
```
[ChunkData (8 bytes)][Chunk payload (up to 256KB)]

ChunkData:
- chunk_index (uint32_t): Chunk index (0-based)
- chunk_size (uint32_t): Size of this chunk's payload
```

The default chunk size is 256KB (262,144 bytes). This provides a good balance between efficiency and network compatibility.

## Example Client (Python)

```python
import asyncio
import websockets
import json
import struct

async def client():
    uri = "ws://localhost:9001"
    
    # For tracking chunked transfers
    chunk_buffers = {}
    CHUNK_MAGIC = 0x4348554E
    
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
        
        # Set checkerboard detection mode
        await websocket.send(json.dumps({
            "cmd": "set_save_mode",
            "mode": "checkerboard",
            "params": {
                "prefix": "calibration",
                "checkerboard_rows": 6,
                "checkerboard_cols": 9,
                "checkerboard_full_res_detection": True,
                "checkerboard_num_threads": 4
            }
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
                # Check for chunked transfer
                if len(message) >= 8:
                    magic = struct.unpack("<I", message[:4])[0]
                    
                    if magic == CHUNK_MAGIC:
                        # Handle chunk start
                        version = struct.unpack("<I", message[4:8])[0]
                        if version == 1 and len(message) >= 36:
                            # Parse chunk header
                            header = struct.unpack("<IIIIIII", message[8:36])
                            frame_id, camera_id, total_chunks, total_size, bytes_per_line, width, height = header
                            
                            # Initialize chunk buffer
                            key = f"{camera_id}_{frame_id}"
                            chunk_buffers[key] = {
                                'chunks': [None] * total_chunks,
                                'received': 0,
                                'total_chunks': total_chunks,
                                'header': (frame_id, camera_id, bytes_per_line, width, height)
                            }
                            continue
                
                # Check if this is chunk data
                if len(message) >= 8:
                    chunk_index, chunk_size = struct.unpack("<II", message[:8])
                    
                    # Find matching chunk buffer
                    for key, buffer in chunk_buffers.items():
                        if buffer['chunks'][chunk_index] is None:
                            # Store chunk
                            buffer['chunks'][chunk_index] = message[8:8+chunk_size]
                            buffer['received'] += 1
                            
                            # Check if complete
                            if buffer['received'] == buffer['total_chunks']:
                                # Reassemble frame
                                frame_data = b''.join(buffer['chunks'])
                                frame_id, camera_id, bytes_per_line, width, height = buffer['header']
                                
                                print(f"Frame {frame_id} from camera {camera_id}: "
                                      f"{width}x{height}, {len(frame_data)} bytes (chunked)")
                                
                                frame_count += 1
                                del chunk_buffers[key]
                            break
                else:
                    # Regular single-message frame
                    header = struct.unpack("<IIIII", message[:20])
                    frame_id, camera_id, bytes_per_line, width, height = header
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
- **Chunked transfers automatically activate for frames larger than 256KB**
- Small delays between chunks prevent network congestion

## Limitations

- Only one client connection is allowed at a time
- All cameras use identical configuration
- Cameras can only be configured once per connection
- Frame saving options must be set before starting cameras
