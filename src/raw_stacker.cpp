#include "raw_stacker.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

namespace {

double nowUs() {
  return std::chrono::duration<double, std::micro>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
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
  j["accumulate_ms"] = c.accumulate_ms;
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

std::optional<uint32_t> RawStacker::arm(StackPlan plan, AckFn on_ack,
                                         FramesFn on_frames) {
  if (plan.camera_ids.empty()) return std::nullopt;
  uint32_t id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) return std::nullopt;
    auto p = std::make_unique<Pending>();
    id = p->id = next_id_++;
    for (uint32_t cam : plan.camera_ids) p->jobs[cam];
    p->plan = std::move(plan);
    p->on_ack = std::move(on_ack);
    p->on_frames = std::move(on_frames);
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
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_) next_id_ = 1;
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
  if (p->on_ack) p->on_ack(std::move(result));
  if (p->on_frames) p->on_frames({});
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
    std::vector<std::shared_ptr<FrameData>> images;
    StackResult result = finish(*p, images);
    // The ack goes out once the images exist — the summed frames are taken,
    // so a robot may move — and ahead of the frames it describes.
    if (p->on_ack) p->on_ack(result);

    std::vector<CaptureFrame> out;
    out.reserve(images.size());
    for (auto& img : images) {
      const double t0 = nowUs();
      // The image is float32 in the frame's payload; detect views it in place.
      cv::Mat view(static_cast<int>(img->height), static_cast<int>(img->width),
                   CV_32FC1, img->data.data());
      CaptureFrame cf;
      cf.sets = raw_stack::detect(view, p->plan.detect);
      cf.detection_kind = detectionKindForMode(p->plan.detect.kind);
      cf.send_image = p->plan.send_image;
      LOG_INFO("RawStacker",
               "Stack " + std::to_string(p->id) + " camera " +
                   std::to_string(img->camera_id) + ": " +
                   std::to_string(cf.sets.size()) + " set(s), detect " +
                   std::to_string((nowUs() - t0) / 1000.0) + " ms");
      cf.frame = std::move(img);
      out.push_back(std::move(cf));
    }
    FramesFn on_frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      on_frames = std::move(pending_->on_frames);
      pending_.reset();
    }
    if (on_frames) on_frames(std::move(out));
  }
}

StackResult RawStacker::finish(Pending& p,
                               std::vector<std::shared_ptr<FrameData>>& images) {
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
    // The image is formed straight into the payload of the frame that goes
    // to the client, and the 12 MB sum is freed before detection: this runs
    // on a 1 GB Pi beside the live pipeline.
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

    frame->camera_id = cam;
    frame->frame_id = job.last.frame_id;
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
    frame->capture_id = p.id;
    images.push_back(std::move(frame));
    LOG_INFO("RawStacker",
             "Stack " + std::to_string(p.id) + " camera " + std::to_string(cam) +
                 ": " + std::to_string(c.frames_stacked) + " frames, accumulate " +
                 std::to_string(c.accumulate_ms) + " ms/frame");
    result.captures.push_back(std::move(c));
  }
  return result;
}
