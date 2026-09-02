#pragma once

// src/python/sync_audio_listener.h — polling bridge for AudioFrameListener.
//
// freenect2_audio has no built-in sync/queue helper (unlike video's
// SyncMultiFrameListener), so this small bridge lives entirely in the
// Python binding rather than in freenect2_audio itself. Deliberately
// latest-frame-wins (not a queue): adequate for polling/monitoring use
// (DOA estimation, level metering) without needing a bounded-queue
// backpressure policy.

#include <libfreenect2/audio_capture.hpp>

#include <mutex>
#include <optional>

namespace pyfreenect2
{

class SyncAudioListener : public libfreenect2::AudioFrameListener
{
public:
  bool onNewAudioFrame(libfreenect2::AudioFrame *frame) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = *frame;
    has_frame_ = true;
    return false; // AudioCapture deletes frame; we've already copied what we need.
  }

  std::optional<libfreenect2::AudioFrame> getLatestFrame()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_frame_)
      return std::nullopt;
    has_frame_ = false;
    return latest_;
  }

private:
  std::mutex mutex_;
  libfreenect2::AudioFrame latest_;
  bool has_frame_ = false;
};

} // namespace pyfreenect2
