# CLAUDE.md - cherupi-v4l2 Project Overview

## Project Summary

cherupi-v4l2 is a high-performance WebSocket server designed for Raspberry Pi 5 that provides real-time streaming of raw camera frames from multiple IMX296 cameras. The server handles camera discovery, configuration, streaming, and optional frame saving with a single-client architecture.

## Key Features

- **Auto-discovery** of all connected IMX296 cameras via V4L2 Media Controller API
- **WebSocket-based control protocol** for remote camera management
- **Real-time frame streaming** with adaptive frame skipping
- **Multiple frame saving modes** (none, buffer, batch)
- **Zero-copy frame transmission** where possible
- **Single-client architecture** with connection blocking

## Architecture Overview

### Core Components

1. **WebSocketServer** (`websocket_server.hpp/cpp`)
   - Main server class using uWebSockets
   - Handles client connections and command routing
   - Enforces single-client policy
   - Manages system state transitions

2. **CameraManager** (`camera_manager.hpp/cpp`)
   - Discovers and manages multiple cameras
   - Configures all cameras with identical settings
   - Coordinates camera lifecycle (configure → start → stop)

3. **Camera** (`camera.hpp/cpp`)
   - Represents individual camera instance
   - Manages V4L2 device and media controller entities
   - Runs capture thread for continuous frame acquisition
   - Provides latest frame for streaming

4. **StreamManager** (`stream_manager.hpp/cpp`)
   - Handles WebSocket frame transmission
   - Implements round-robin streaming for multiple cameras
   - Manages backpressure and frame skipping
   - Multiplexes multiple camera streams over single connection

5. **FrameSaver** (`frame_saver.hpp/cpp`)
   - Optional frame saving functionality
   - Three modes: none, buffer (memory), batch (disk)
   - Multi-threaded batch writing with O_DIRECT

6. **MediaDevice** (`media_device.hpp/cpp`)
   - Wraps Linux Media Controller API
   - Configures sensor pipeline (sensor → CSI2 → video)
   - Sets formats, crops, and enables links

7. **V4L2Device** (`v4l2_device.hpp/cpp`)
   - Wraps V4L2 video device operations
   - Manages memory-mapped buffers
   - Captures frames in SRGGB10P format

## Protocol Specification

### WebSocket Connection
- Port: 9001
- Only one client allowed at a time
- Text messages for commands (JSON)
- Binary messages for frame data

### Client → Server Commands

```json
// Camera discovery
{"cmd": "discover"}

// Configure all cameras
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

// Start streaming specific camera
{"cmd": "start_stream", "camera_id": 0}

// Stop streaming specific camera
{"cmd": "stop_stream", "camera_id": 0}

// Stop all cameras
{"cmd": "stop_cameras"}
```

### Server → Client Responses

Text messages (JSON):
```json
{"type": "discovery", "cameras": [{"id": 0, "type": "IMX296"}, ...]}
{"type": "status", "message": "..."}
{"type": "error", "message": "..."}
```

Binary frame format:
```
[FrameHeader (20 bytes)][Raw SRGGB10P data]

FrameHeader structure (packed):
- frame_id (uint32_t): Frame sequence number
- camera_id (uint32_t): Camera identifier
- bytes_per_line (uint32_t): Frame stride
- width (uint32_t): Frame width in pixels
- height (uint32_t): Frame height in pixels
```

## State Machine

The system follows these state transitions:

```
IDLE → CONFIGURED → RUNNING
         ↑            ↓
         ←────────────
```

- **IDLE**: Initial state, cameras discovered but not configured
- **CONFIGURED**: Cameras configured but not capturing
- **RUNNING**: Cameras actively capturing, streaming possible

## Key Design Decisions

1. **Single Client Architecture**: Simplifies state management and resource allocation
2. **Identical Camera Configuration**: All cameras use same resolution/format
3. **Latest Frame Streaming**: Only most recent frame kept for streaming (drops older frames)
4. **Separate Capture and Stream Threads**: Decouples frame acquisition from network transmission
5. **Round-Robin Streaming**: Fair bandwidth distribution across cameras
6. **Adaptive Frame Skipping**: Maintains real-time performance under network congestion

## File Organization

### Core Server Files
- `main.cpp`: Entry point with signal handling
- `websocket_server.*`: WebSocket server implementation
- `types.hpp`: Common data structures and protocol constants

### Camera Management
- `camera_manager.*`: Multi-camera coordination
- `camera.*`: Individual camera control
- `stream_manager.*`: Frame streaming logic

### V4L2 Integration
- `media_device.*`: Media Controller API wrapper
- `v4l2_device.*`: V4L2 video device wrapper

### Utilities
- `frame_saver.*`: Optional frame saving
- `v4l2_capture.cpp`: Standalone capture utility (not part of server)

### Build System
- `CMakeLists.txt`: CMake configuration
- `.gitignore`: Git ignore rules
- `README.md`: User documentation

## Dependencies

### System Requirements
- Raspberry Pi 5 with IMX296 cameras
- Linux with V4L2 and Media Controller support
- Root/sudo access for camera control

### Build Dependencies
- CMake 3.14+
- C++17 compiler
- OpenSSL development libraries
- zlib development libraries

### Auto-downloaded Dependencies
- uWebSockets (with uSockets)
- nlohmann/json

## Memory and Performance Considerations

1. **Frame Size**: SRGGB10P format uses 5 bytes per 4 pixels
   - Default 1456×1088 resolution = ~1.98 MB per frame

2. **Buffer Management**:
   - V4L2 uses 4 memory-mapped buffers by default
   - Latest frame kept for streaming (1 per camera)
   - Optional frame saving buffers depend on mode

3. **Network Optimization**:
   - Binary frame transmission (no encoding overhead)
   - Single frame in network pipeline per camera
   - Backpressure handling to prevent memory buildup

## Common Operations Flow

1. **Connection and Discovery**:
   - Client connects to ws://host:9001
   - Client sends "discover" command
   - Server responds with camera list

2. **Configuration and Start**:
   - Client sends "configure" with parameters
   - Client optionally sets save mode
   - Client sends "start_cameras"
   - All cameras begin capturing

3. **Streaming**:
   - Client sends "start_stream" for each desired camera
   - Server begins sending binary frames
   - Client can start/stop streams dynamically
   - Cameras continue capturing regardless of streaming state

4. **Shutdown**:
   - Client sends "stop_cameras"
   - Server stops all capture threads
   - Frame saving completes (if enabled)
   - Resources are freed

## Error Handling

- Commands rejected if system state doesn't allow operation
- Individual camera failures don't crash the system
- Network disconnection triggers graceful cleanup
- Frame drops tracked but don't interrupt streaming

## Future Enhancement Opportunities

1. Multi-client support with session management
2. Per-camera configuration options
3. Frame compression/encoding options
4. Hardware-accelerated image processing
5. Camera hot-plug support
6. Web UI for configuration
7. Recording triggered by events
8. Integration with computer vision pipelines
