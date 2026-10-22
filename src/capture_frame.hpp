#pragma once

#include <memory>
#include <vector>

#include "frame_processor.hpp"  // CornerSet
#include "types.hpp"

// One frame produced by an on-demand capture (trigger_capture or
// stack_capture) on its way to the commander that asked for it. Unlike live
// frames these are queued, not latest-wins: each goes out exactly once, in
// order, whatever the subscription, max_fps, subsample or header-only state.
struct CaptureFrame {
  // frame_kind is TRIGGER or STACK and capture_id the request's id.
  std::shared_ptr<const FrameData> frame;
  // The process mode's detection on this frame, in the frame's own pixels.
  std::vector<CornerSet> sets;
  DetectionKind detection_kind = DetectionKind::NONE;
  // The request's send_image: false sends the header and detection block
  // only.
  bool send_image = true;
};
