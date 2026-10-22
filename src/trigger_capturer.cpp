#include "trigger_capturer.hpp"

#include <algorithm>

nlohmann::json triggerResultJson(const TriggerResult& result) {
  nlohmann::json j;
  j["type"] = Protocol::TYPE_TRIGGER_RESULT;
  j["trigger_id"] = result.trigger_id;
  j["cancelled"] = result.cancelled;
  j["captures"] = nlohmann::json::array();
  for (const auto& c : result.captures)
    j["captures"].push_back({{"camera_id", c.camera_id}, {"frame_id", c.frame_id}});
  return j;
}

TriggerCapturer::TriggerCapturer(DetectFn detect) : detect_(std::move(detect)) {
  worker_ = std::thread(&TriggerCapturer::workerLoop, this);
}

TriggerCapturer::~TriggerCapturer() {
  cancel();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  work_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

std::optional<uint32_t> TriggerCapturer::arm(TriggerPlan plan, AckFn on_ack,
                                              FramesFn on_frames) {
  if (plan.camera_ids.empty()) return std::nullopt;
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_ || job_) return std::nullopt;
  auto p = std::make_unique<Pending>();
  p->id = next_id_++;
  const int skip = std::max(0, plan.skip_frames);
  for (uint32_t cam : plan.camera_ids) p->waiting[cam] = skip;
  p->result.trigger_id = p->id;
  p->plan = std::move(plan);
  p->on_ack = std::move(on_ack);
  p->on_frames = std::move(on_frames);
  const uint32_t id = p->id;
  pending_ = std::move(p);
  armed_ = true;
  LOG_INFO("TriggerCapturer",
           "Trigger " + std::to_string(id) + " armed for " +
               std::to_string(pending_->waiting.size()) +
               " camera(s), skipping " + std::to_string(skip) +
               " frame(s) each");
  return id;
}

bool TriggerCapturer::hasPending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_ != nullptr;
}

void TriggerCapturer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_) next_id_ = 1;
}

std::unique_ptr<TriggerCapturer::Pending> TriggerCapturer::takeIfDone() {
  if (!pending_ || pending_->copies_in_flight > 0) return nullptr;
  if (!pending_->waiting.empty() && !pending_->cancelled) return nullptr;
  armed_ = false;
  return std::move(pending_);
}

void TriggerCapturer::onFrame(const FrameData& frame) {
  if (!armed_.load(std::memory_order_acquire)) return;
  uint32_t id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->cancelled) return;
    auto it = pending_->waiting.find(frame.camera_id);
    // Not part of this trigger, or already served by an earlier frame.
    if (it == pending_->waiting.end()) return;
    if (it->second > 0) {  // a settling frame the client asked to skip
      --it->second;
      return;
    }
    pending_->waiting.erase(it);
    pending_->result.captures.push_back({frame.camera_id, frame.frame_id});
    ++pending_->copies_in_flight;
    id = pending_->id;
  }

  // Copy the multi-MB payload outside the lock so a second camera's
  // completion thread isn't held up behind this one's memcpy.
  auto copy = std::make_shared<FrameData>(frame);
  copy->frame_kind = FrameKind::TRIGGER;
  copy->capture_id = id;

  std::unique_ptr<Pending> done;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // cancel() leaves a trigger with copies in flight in place, so this is
    // still the trigger the frame was claimed for.
    pending_->frames.push_back(std::move(copy));
    --pending_->copies_in_flight;
    done = takeIfDone();
  }
  if (done) complete(std::move(done));
}

void TriggerCapturer::cancel() {
  std::unique_ptr<Pending> done;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) return;
    pending_->cancelled = true;
    // With copies still in flight the last copier completes it instead.
    done = takeIfDone();
  }
  if (done) complete(std::move(done));
}

void TriggerCapturer::complete(std::unique_ptr<Pending> p) {
  p->result.cancelled = p->cancelled;
  if (p->cancelled) {
    LOG_WARN("TriggerCapturer",
             "Trigger " + std::to_string(p->id) +
                 " cancelled: capture stopped with " +
                 std::to_string(p->waiting.size()) + " camera(s) still to deliver");
  }
  // The ack first: it means "captured", which is what a robot loop waits on
  // before moving, and it goes out ahead of the frames it describes.
  if (p->on_ack) p->on_ack(p->result);
  auto job = std::make_unique<Job>();
  job->plan = std::move(p->plan);
  job->frames = std::move(p->frames);
  job->on_frames = std::move(p->on_frames);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job_ = std::move(job);
  }
  work_cv_.notify_all();
}

void TriggerCapturer::workerLoop() {
  for (;;) {
    std::unique_ptr<Job> job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [this] { return stop_ || job_ != nullptr; });
      if (!job_) return;  // stopping, nothing left to deliver
      job = std::move(job_);
    }
    std::vector<CaptureFrame> out;
    out.reserve(job->frames.size());
    for (auto& frame : job->frames) {
      CaptureFrame cf;
      if (job->plan.detection_kind != DetectionKind::NONE && detect_)
        cf.sets = detect_(*frame);
      cf.frame = std::move(frame);
      cf.detection_kind = job->plan.detection_kind;
      cf.send_image = job->plan.send_image;
      out.push_back(std::move(cf));
    }
    if (job->on_frames) job->on_frames(std::move(out));
  }
}
