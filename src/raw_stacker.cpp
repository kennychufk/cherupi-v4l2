#include "raw_stacker.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace {

double nowUs() {
  return std::chrono::duration<double, std::micro>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

const char* detectName(ProcessMode mode) {
  switch (mode) {
    case ProcessMode::CHECKERBOARD:
      return "checkerboard";
    case ProcessMode::CHECKERBOARD2X2:
      return "checkerboard2x2";
    case ProcessMode::ARUCO:
      return "aruco";
    case ProcessMode::ARUCO2X2:
      return "aruco2x2";
    default:
      return "none";
  }
}

nlohmann::json captureJson(const StackResult::Capture& c) {
  nlohmann::json j;
  j["camera_id"] = c.camera_id;
  if (!c.error.empty()) j["error"] = c.error;
  j["frames_stacked"] = c.frames_stacked;
  j["frames_rejected"] = c.frames_rejected;
  j["settle_frames"] = c.settle_frames;
  j["first_frame_id"] = c.first_frame_id;
  j["last_frame_id"] = c.last_frame_id;
  j["first_timestamp_us"] = c.first_timestamp_us;
  j["last_timestamp_us"] = c.last_timestamp_us;
  j["exposure_time"] = c.exposure_time_us;
  j["analogue_gain"] = c.analogue_gain;
  j["black_level"] = c.black_level;
  j["bit_depth"] = c.bit_depth;
  j["white_fraction"] = c.white_fraction;
  j["peak_fraction"] = c.peak_fraction;
  j["clipped_fraction"] = c.clipped_fraction;
  j["width"] = c.width;
  j["height"] = c.height;
  j["detect"] = detectName(c.detect);
  j["sets"] = nlohmann::json::array();
  for (const auto& set : c.sets) {
    nlohmann::json s;
    s["set_id"] = set.set_id;
    if (set.marker_id >= 0) s["marker_id"] = set.marker_id;
    s["corners"] = nlohmann::json::array();
    for (const auto& p : set.corners) s["corners"].push_back({p.x, p.y});
    j["sets"].push_back(std::move(s));
  }
  j["accumulate_ms"] = c.accumulate_ms;
  j["detect_ms"] = c.detect_ms;
  j["filename"] = c.filename;
  return j;
}

}  // namespace

nlohmann::json stackResultJson(const StackResult& result) {
  nlohmann::json j;
  j["type"] = Protocol::TYPE_STACK_RESULT;
  j["stack_id"] = result.stack_id;
  j["cancelled"] = result.cancelled;
  j["captures"] = nlohmann::json::array();
  for (const auto& c : result.captures) j["captures"].push_back(captureJson(c));
  return j;
}

RawStacker::RawStacker(OverrideFn apply, RestoreFn restore)
    : apply_(std::move(apply)), restore_(std::move(restore)) {
  worker_ = std::thread(&RawStacker::workerLoop, this);
}

RawStacker::~RawStacker() {
  cancel();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  work_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

std::optional<uint32_t> RawStacker::arm(
    StackPlan plan, std::function<void(StackResult)> on_complete) {
  if (plan.camera_ids.empty()) return std::nullopt;
  uint32_t id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) return std::nullopt;
    auto p = std::make_unique<Pending>();
    id = p->id = next_id_++;
    for (uint32_t cam : plan.camera_ids) p->jobs[cam];
    p->plan = std::move(plan);
    p->on_complete = std::move(on_complete);
    pending_ = std::move(p);
    if (apply_ && (pending_->plan.exposure_time_us || pending_->plan.analogue_gain)) {
      for (const auto& [cam, job] : pending_->jobs)
        apply_(cam, pending_->plan.exposure_time_us, pending_->plan.analogue_gain);
    }
  }
  return id;
}

bool RawStacker::hasPending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_ != nullptr;
}

void RawStacker::reset() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) next_id_ = 1;
  }
  std::lock_guard<std::mutex> lock(published_mutex_);
  published_.clear();
}

void RawStacker::onRawFrame(const RawFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_ || pending_->finishing) return;
  auto it = pending_->jobs.find(frame.camera_id);
  if (it == pending_->jobs.end() || it->second.phase == Phase::DONE) return;
  advance(it->second, frame);
  if (it->second.phase == Phase::DONE && restore_ &&
      (pending_->plan.exposure_time_us || pending_->plan.analogue_gain))
    restore_(frame.camera_id);

  for (const auto& [cam, job] : pending_->jobs)
    if (job.phase != Phase::DONE) return;
  pending_->finishing = true;
  work_cv_.notify_all();
}

void RawStacker::advance(CameraJob& job, const RawFrame& frame) {
  const StackPlan& plan = pending_->plan;
  RawFrame meta = frame;
  meta.data = nullptr;

  if (job.phase == Phase::SETTLING) {
    if (raw_stack::exposureMatches(plan.exposure_time_us, frame.exposure_time_us) &&
        raw_stack::gainMatches(plan.analogue_gain, frame.analogue_gain)) {
      job.phase = plan.skip_frames > 0 ? Phase::SKIPPING : Phase::STACKING;
    } else if (++job.settle_frames > kSettleMaxFrames) {
      std::ostringstream msg;
      msg << "exposure/gain did not settle within " << kSettleMaxFrames
          << " frames: asked";
      if (plan.exposure_time_us) msg << " " << *plan.exposure_time_us << " us";
      if (plan.analogue_gain) msg << " gain " << *plan.analogue_gain;
      msg << ", camera reports " << frame.exposure_time_us << " us gain "
          << frame.analogue_gain
          << " (an exposure longer than a locked frame duration is clamped)";
      job.error = msg.str();
      job.last = meta;
      job.phase = Phase::DONE;
      return;
    } else {
      return;
    }
  }

  if (job.phase == Phase::SKIPPING) {
    if (job.skipped++ < plan.skip_frames) return;
    job.phase = Phase::STACKING;
  }

  // STACKING. Every frame must match the first one summed. Under auto
  // exposure (nothing pinned) the AGC may still be moving — right after a
  // previous stack released its pin, say — so a change restarts the sum from
  // the new frame rather than mixing settings into one mean; the camera fails
  // only once a whole window's worth of frames has been thrown away.
  if (job.acc.frames() > 0 &&
      (!raw_stack::exposureMatches(job.first.exposure_time_us,
                                   frame.exposure_time_us) ||
       !raw_stack::gainMatches(std::isfinite(job.first.analogue_gain)
                                   ? std::optional<float>(job.first.analogue_gain)
                                   : std::nullopt,
                               frame.analogue_gain))) {
    job.rejected += job.acc.frames();
    if (job.rejected > std::max(plan.num_frames, kSettleMaxFrames)) {
      job.error = "exposure/gain kept changing: " + std::to_string(job.rejected) +
                  " frames discarded (pin exposure_time to stack under auto "
                  "exposure)";
      job.phase = Phase::DONE;
      return;
    }
    job.acc.reset(frame.width, frame.height);
    job.clipped_max = 0;
    job.accumulate_us = 0.0;
  }
  if (job.acc.frames() == 0) {
    job.acc.reset(frame.width, frame.height);
    job.first = meta;
  }
  const double t0 = nowUs();
  const uint16_t clip = static_cast<uint16_t>(
      raw_stack::fullScaleCode(frame.bit_depth) -
      (raw_stack::fullScaleCode(frame.bit_depth) >> 6));  // within ~1.6 %
  job.clipped_max = std::max(job.clipped_max, job.acc.add(frame, clip));
  job.accumulate_us += nowUs() - t0;
  job.last = meta;
  if (job.acc.frames() >= plan.num_frames) job.phase = Phase::DONE;
}

void RawStacker::cancel() {
  std::unique_ptr<Pending> p;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->finishing) return;
    p = std::move(pending_);
    const bool pinned = p->plan.exposure_time_us || p->plan.analogue_gain;
    for (const auto& [cam, job] : p->jobs)
      if (job.phase != Phase::DONE && pinned && restore_) restore_(cam);
  }
  StackResult result;
  result.stack_id = p->id;
  result.cancelled = true;
  for (const auto& [cam, job] : p->jobs) {
    StackResult::Capture c;
    c.camera_id = cam;
    c.error = job.error.empty() ? "cancelled" : job.error;
    c.frames_stacked = job.acc.frames();
    c.settle_frames = job.settle_frames;
    result.captures.push_back(std::move(c));
  }
  if (p->on_complete) p->on_complete(std::move(result));
}

void RawStacker::workerLoop() {
  for (;;) {
    Pending* p = nullptr;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [this] {
        return stop_ || (pending_ && pending_->finishing);
      });
      if (stop_) return;
      p = pending_.get();
    }
    // `finishing` keeps onRawFrame, arm and cancel off the pending stack, so
    // it can be read here without the lock while detection runs.
    StackResult result = finish(*p);
    std::function<void(StackResult)> done;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done = std::move(pending_->on_complete);
      pending_.reset();
    }
    // The result goes out before the image is published, so a client sees
    // the stack_result ahead of the frame it describes.
    if (done) done(result);
    publish(result);
  }
}

StackResult RawStacker::finish(Pending& p) {
  StackResult result;
  result.stack_id = p.id;
  for (auto& [cam, job] : p.jobs) {
    StackResult::Capture c;
    c.camera_id = cam;
    c.error = job.error;
    c.frames_stacked = job.acc.frames();
    c.frames_rejected = job.rejected;
    c.settle_frames = job.settle_frames;
    c.first_frame_id = job.first.frame_id;
    c.last_frame_id = job.last.frame_id;
    c.first_timestamp_us = job.first.timestamp_us;
    c.last_timestamp_us = job.last.timestamp_us;
    c.exposure_time_us = job.first.exposure_time_us;
    c.analogue_gain = std::isfinite(job.first.analogue_gain) ? job.first.analogue_gain : 0.f;
    c.bit_depth = job.first.bit_depth;
    c.detect = p.plan.detect.kind;
    if (!c.error.empty() || job.acc.frames() == 0) {
      if (c.error.empty()) c.error = "no frames stacked";
      result.captures.push_back(std::move(c));
      continue;
    }

    // Black level: libcamera's, or the conventional 1/16 of full scale
    // (256 of 4095 at 12 bits) when the IPA did not report one.
    const float full = raw_stack::fullScaleCode(job.first.bit_depth);
    const float black = std::isfinite(job.first.black_level)
                            ? job.first.black_level
                            : full / 16.f;
    c.black_level = black;
    // The image is formed straight into the payload of the frame the
    // streamers will share, and the 12 MB sum is freed before detection:
    // this runs on a 1 GB Pi beside the live pipeline.
    auto frame = std::make_shared<FrameData>();
    cv::Mat img = raw_stack::meanImage(job.acc, black, frame->data);
    c.width = static_cast<uint32_t>(img.cols);
    c.height = static_cast<uint32_t>(img.rows);
    const float range = std::max(1.f, full - black);
    c.white_fraction = raw_stack::percentile(img, 0.995) / range;
    c.peak_fraction = (static_cast<float>(job.acc.peak()) - black) / range;
    c.clipped_fraction = static_cast<float>(
        static_cast<double>(job.clipped_max) /
        (4.0 * static_cast<double>(job.acc.width()) * job.acc.height()));
    c.accumulate_ms = job.accumulate_us / job.acc.frames() / 1000.0;
    job.acc.release();

    const double t0 = nowUs();
    std::vector<CornerSet> binned = raw_stack::detect(img, p.plan.detect);
    c.detect_ms = (nowUs() - t0) / 1000.0;
    c.sets = binned;
    for (auto& set : c.sets)
      for (auto& pt : set.corners) pt = raw_stack::binnedToFull(pt);

    if (p.plan.save_image) {
      std::error_code ec;
      std::filesystem::create_directories(p.plan.output_dir, ec);
      const std::string base = p.plan.output_dir + "/stack" +
                               std::to_string(p.id) + "-cam" +
                               std::to_string(cam);
      c.filename = base + ".npy";
      if (raw_stack::writeNpy(c.filename, img)) {
        // A sidecar with the same record the client received, so the image
        // can be analysed offline without the session log.
        std::ofstream(base + ".json") << captureJson(c).dump(1) << "\n";
      } else {
        LOG_ERROR("RawStacker", "Failed to write " + c.filename);
        c.filename.clear();
        c.error = "failed to write the stacked image to " + p.plan.output_dir;
      }
    }

    if (p.plan.publish) {
      frame->camera_id = cam;
      frame->frame_id = p.id;
      frame->width = c.width;
      frame->height = c.height;
      frame->bytes_per_line = c.width * sizeof(float);
      frame->pixel_format = PIXEL_FORMAT_STACK_F32;
      frame->timestamp_us = job.last.timestamp_us;
      frame->frame_duration_us = job.last.frame_duration_us;
      frame->lens_position = job.last.lens_position;
      frame->af_state = job.last.af_state;
      frame->focus_fom = job.last.focus_fom;
      frame->frame_kind = FrameKind::STACK;
      c.image = std::move(frame);
      c.image_sets = std::move(binned);
    }
    LOG_INFO("RawStacker",
             "Stack " + std::to_string(p.id) + " camera " + std::to_string(cam) +
                 ": " + std::to_string(c.frames_stacked) + " frames, " +
                 std::to_string(c.sets.size()) + " set(s), accumulate " +
                 std::to_string(c.accumulate_ms) + " ms/frame, detect " +
                 std::to_string(c.detect_ms) + " ms");
    result.captures.push_back(std::move(c));
  }
  return result;
}

void RawStacker::publish(const StackResult& result) {
  bool any = false;
  {
    std::lock_guard<std::mutex> lock(published_mutex_);
    for (const auto& c : result.captures) {
      if (!c.image) continue;
      Published& slot = published_[c.camera_id];
      slot.frame = c.image;
      slot.sets = c.image_sets;
      slot.detection_kind = static_cast<uint8_t>(detectionKindForMode(c.detect));
      slot.seq = ++published_seq_;
      any = true;
    }
  }
  if (any && on_published_) on_published_();
}

bool RawStacker::getStackedFrameForStreaming(
    uint32_t camera_id, uint64_t& last_seen_seq,
    std::shared_ptr<const FrameData>& frame, std::vector<CornerSet>& sets,
    uint8_t& detection_kind) {
  std::lock_guard<std::mutex> lock(published_mutex_);
  auto it = published_.find(camera_id);
  if (it == published_.end() || !it->second.frame ||
      it->second.seq <= last_seen_seq)
    return false;
  frame = it->second.frame;
  sets = it->second.sets;
  detection_kind = it->second.detection_kind;
  last_seen_seq = it->second.seq;
  return true;
}
