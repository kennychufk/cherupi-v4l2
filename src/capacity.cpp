#include "capacity.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>

namespace capacity {
namespace {

constexpr size_t kMiB = 1024ull * 1024ull;

// Size of the O_DIRECT probe write. Large enough to outlast an SD card's burst
// cache (a 4 MiB probe reads ~3x optimistic on the cards we ship on), small
// enough that the one-time cost stays around a second on a slow card.
constexpr size_t kProbeBytes = 32 * kMiB;
constexpr size_t kProbeAlignment = 4096;

std::mutex g_probe_mutex;
std::map<std::string, size_t> g_probe_cache;  // guarded by g_probe_mutex

}  // namespace

size_t frameBytes(uint32_t width, uint32_t height) {
  return static_cast<size_t>(width) * static_cast<size_t>(height) * 3 / 2;
}

size_t availableMemoryBytes() {
  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo) return 0;
  std::string line;
  while (std::getline(meminfo, line)) {
    if (line.rfind("MemAvailable:", 0) != 0) continue;
    std::istringstream iss(line.substr(std::strlen("MemAvailable:")));
    size_t kb = 0;
    if (iss >> kb) return kb * 1024;
    return 0;
  }
  return 0;
}

size_t defaultBacklogBudgetBytes(size_t mem_available) {
  constexpr size_t kFloor = 64 * kMiB;
  constexpr size_t kCap = 2048 * kMiB;
  if (mem_available == 0) return kFloor;  // unknown: be conservative
  return std::clamp(mem_available / 2, kFloor, kCap);
}

size_t diskWriteBytesPerSecond(const std::string& dir) {
  {
    std::lock_guard<std::mutex> lock(g_probe_mutex);
    auto it = g_probe_cache.find(dir);
    if (it != g_probe_cache.end()) return it->second;
  }

  size_t rate = 0;
  const std::string path = (std::filesystem::path(dir) / ".cherupi-write-probe").string();

  // O_DIRECT so the measurement is the device's sustained rate and not the
  // speed of memcpy into the page cache — the frames themselves are written
  // O_DIRECT too (writerThreadFunc), so this matches the real write path.
  int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_DIRECT, 0644);
  if (fd >= 0) {
    void* buffer = nullptr;
    if (posix_memalign(&buffer, kProbeAlignment, kProbeBytes) == 0) {
      std::memset(buffer, 0, kProbeBytes);
      const auto start = std::chrono::steady_clock::now();
      const ssize_t written = ::write(fd, buffer, kProbeBytes);
      ::fsync(fd);
      const auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
              .count();
      if (written > 0 && elapsed > 0.0) {
        rate = static_cast<size_t>(static_cast<double>(written) / elapsed);
      }
      ::free(buffer);
    }
    ::close(fd);
  }
  std::error_code ec;
  std::filesystem::remove(path, ec);

  if (rate > 0) {
    LOG_INFO("Capacity", "Measured write throughput of " + dir + ": " +
                             formatRate(rate));
  } else {
    LOG_WARN("Capacity",
             "Could not measure write throughput of " + dir +
                 "; skipping the disk feasibility check");
  }

  std::lock_guard<std::mutex> lock(g_probe_mutex);
  g_probe_cache[dir] = rate;
  return rate;
}

Demand computeDemand(uint32_t width, uint32_t height, size_t camera_count,
                     int64_t frame_duration_us,
                     int64_t hw_min_frame_duration_us) {
  Demand d;
  d.bytes_per_frame = frameBytes(width, height);
  d.camera_count = camera_count;

  // An unset FrameDurationLimits lock means libcamera picks the cadence, so
  // the pipeline has to survive the sensor's fastest mode.
  const int64_t period_us =
      frame_duration_us > 0 ? frame_duration_us : hw_min_frame_duration_us;
  if (period_us <= 0) return d;  // cadence unknown; callers abstain

  d.frames_per_second = 1e6 / static_cast<double>(period_us);
  d.bytes_per_second = static_cast<size_t>(
      static_cast<double>(d.bytes_per_frame) * d.frames_per_second *
      static_cast<double>(camera_count == 0 ? 1 : camera_count));
  return d;
}

Verdict checkDiskThroughput(ProcessMode mode, bool save_frames,
                            const Demand& demand,
                            size_t disk_bytes_per_second) {
  // Only BATCH writes every captured frame continuously. Detector modes write
  // only frames that detected something, which has no rate we can bound
  // ahead of time.
  if (mode != ProcessMode::BATCH || !save_frames) return {};
  if (demand.bytes_per_second == 0 || disk_bytes_per_second == 0) return {};
  if (demand.bytes_per_second <= disk_bytes_per_second) return {};

  const double shortfall = static_cast<double>(demand.bytes_per_second) /
                           static_cast<double>(disk_bytes_per_second);
  std::ostringstream oss;
  oss << "batch mode at " << formatBytes(demand.bytes_per_frame) << "/frame x "
      << std::fixed << std::setprecision(2) << demand.frames_per_second
      << " fps";
  if (demand.camera_count > 1) oss << " x " << demand.camera_count << " cameras";
  oss << " needs " << formatRate(demand.bytes_per_second)
      << ", but this filesystem sustains only "
      << formatRate(disk_bytes_per_second) << " ("
      << std::fixed << std::setprecision(1) << shortfall
      << "x too slow). The write queue would grow until the backlog budget is "
         "exhausted and frames start dropping. Lower the frame rate or the "
         "resolution, take frames on demand with trigger_capture instead, write "
         "to faster storage, or set allow_overcommit to start anyway.";
  return {false, oss.str()};
}

Verdict checkBufferHeadroom(ProcessMode mode, bool save_frames,
                            const Demand& demand, size_t budget_bytes) {
  if (mode != ProcessMode::BUFFER || !save_frames) return {};
  if (demand.bytes_per_frame == 0 || budget_bytes == 0) return {};

  const size_t frames = budget_bytes / demand.bytes_per_frame;
  if (demand.bytes_per_second == 0) return {};  // cadence unknown

  const double seconds = static_cast<double>(budget_bytes) /
                         static_cast<double>(demand.bytes_per_second);

  // BUFFER never drains before stop(), so its whole capacity is this window.
  // Under two seconds means the budget cannot hold a meaningful capture and
  // the operator has almost certainly picked the wrong mode.
  if (seconds >= 2.0) return {};

  std::ostringstream oss;
  oss << "buffer mode holds every frame in RAM until stop, but the backlog "
         "budget of "
      << formatBytes(budget_bytes) << " fits only " << frames << " frame(s) at "
      << formatBytes(demand.bytes_per_frame) << " each — " << std::fixed
      << std::setprecision(1) << seconds << " s of capture at "
      << std::setprecision(2) << demand.frames_per_second
      << " fps. Use 'batch' to stream to disk, take frames on demand with "
         "trigger_capture, lower the resolution, or set allow_overcommit to "
         "start anyway.";
  return {false, oss.str()};
}

std::string formatBytes(size_t bytes) {
  static const char* units[] = {"B", "KB", "MB", "GB", "TB"};
  double value = static_cast<double>(bytes);
  int unit = 0;
  while (value >= 1000.0 && unit < 4) {
    value /= 1000.0;
    ++unit;
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(unit == 0 ? 0 : 1) << value << " "
      << units[unit];
  return oss.str();
}

std::string formatRate(size_t bytes_per_second) {
  return formatBytes(bytes_per_second) + "/s";
}

}  // namespace capacity
