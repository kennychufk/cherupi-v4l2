#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "types.hpp"

// Resource arithmetic for the capture pipeline: how much RAM a mode's frame
// backlog may hold, and whether the configured mode can actually keep up with
// the camera on this hardware.
//
// The motivating failure: at 4608x2592 YUV420 a frame is 17.9 MB, so a mode
// that retains every frame (BUFFER) or writes every frame to a sink slower
// than capture (BATCH on an SD card) builds an unbounded backlog. On a 1 GB Pi
// the box then swap-thrashes, the libcamera completion thread misses the RPi
// dequeue watchdog (5x frame duration, floored at 1 s), and the PiSP frontend
// dies with "Camera frontend has timed out!" — permanently, and silently.
// Bounding the backlog turns that into dropped frames; checking feasibility up
// front turns it into an error the operator can act on.
//
// Everything except availableMemoryBytes() and diskWriteBytesPerSecond() is
// pure arithmetic with no I/O, so the interesting logic is unit-testable.
namespace capacity {

// Bytes one YUV420 frame of these dimensions occupies (Y + U + V = w*h*3/2).
size_t frameBytes(uint32_t width, uint32_t height);

// MemAvailable from /proc/meminfo, in bytes. 0 when it cannot be read.
size_t availableMemoryBytes();

// Backlog budget to use when ProcessConfig::backlog_max_bytes is 0: half of
// `mem_available`, clamped to [64 MiB, 2 GiB]. Half leaves room for the
// mmapped DMA buffers, the streaming path's own copies, and the page cache
// the writer pool needs; the floor keeps the budget usable on a tiny box, and
// the cap stops a large-memory host from buffering absurdly long.
size_t defaultBacklogBudgetBytes(size_t mem_available);

// Sustained write throughput of the filesystem holding `dir`, in bytes/s,
// measured with a short O_DIRECT write (page cache bypassed, so this is the
// real sustained rate rather than a burst into RAM). Measured once per
// directory and cached for the process lifetime; returns 0 if the probe could
// not run (unwritable directory, O_DIRECT unsupported).
size_t diskWriteBytesPerSecond(const std::string& dir);

// What the capture pipeline demands of its sink, given the stream geometry and
// cadence. `frames_per_second` is per camera; `bytes_per_second` is the total
// across all cameras, which is what the disk and the backlog actually see.
struct Demand {
  size_t bytes_per_frame = 0;
  double frames_per_second = 0.0;
  size_t camera_count = 0;
  size_t bytes_per_second = 0;
};

// `frame_duration_us` is the locked FrameDurationLimits value; 0 (unset) means
// the cadence is unknown, in which case `hw_min_frame_duration_us` — the
// sensor's fastest mode — is used, since that is the worst case the pipeline
// must survive. Both 0 ⇒ frames_per_second and bytes_per_second are 0, i.e.
// "unknown", and every check below abstains.
Demand computeDemand(uint32_t width, uint32_t height, size_t camera_count,
                     int64_t frame_duration_us,
                     int64_t hw_min_frame_duration_us);

// Outcome of one feasibility check. `feasible` false means the mode provably
// cannot keep up and start_cameras should refuse (unless allow_overcommit).
// `reason` is operator-facing and always names the numbers that decided it.
// A check with nothing to say (unknown cadence, mode that doesn't write, no
// measurement available) returns feasible with an empty reason.
struct Verdict {
  bool feasible = true;
  std::string reason;
};

// Can the disk keep up with a mode that writes every captured frame? Only
// BATCH does that continuously; the detector modes write only what they
// detect, and TRIGGER only what is asked for, so both abstain here.
Verdict checkDiskThroughput(ProcessMode mode, bool save_frames,
                            const Demand& demand,
                            size_t disk_bytes_per_second);

// How long can BUFFER run before its budget is exhausted? BUFFER never drains
// until stop(), so its capacity is a fixed number of frames — this reports it
// and refuses only when the budget cannot hold even a couple of seconds of
// capture, which is almost certainly a misconfiguration rather than an intent.
Verdict checkBufferHeadroom(ProcessMode mode, bool save_frames,
                            const Demand& demand, size_t budget_bytes);

// Human-readable byte count ("17.9 MB", "967 MB", "1.9 GB"), for log lines and
// error messages that an operator has to act on.
std::string formatBytes(size_t bytes);

// Human-readable rate ("21.5 MB/s").
std::string formatRate(size_t bytes_per_second);

}  // namespace capacity
