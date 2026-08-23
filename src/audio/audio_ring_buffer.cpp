/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file audio_ring_buffer.cpp Timestamp-queryable rolling audio window. */

#include <libfreenect2/audio_ring_buffer.hpp>

#include <deque>
#include <mutex>

namespace libfreenect2
{

class AudioRingBuffer::Impl
{
public:
  explicit Impl(uint64_t retention_us) : retention_us_(retention_us) {}

  uint64_t retention_us_;
  mutable std::mutex mutex_;
  std::deque<AudioFrame> frames_;

  void trim(uint64_t now_us)
  {
    while (!frames_.empty() &&
           now_us - frames_.front().timestamp_us > retention_us_)
      frames_.pop_front();
  }
};

AudioRingBuffer::AudioRingBuffer(uint64_t retention_us) :
  impl_(std::make_unique<Impl>(retention_us))
{
}

AudioRingBuffer::~AudioRingBuffer() = default;

void AudioRingBuffer::push(const AudioFrame &frame)
{
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  impl_->frames_.push_back(frame);
  impl_->trim(frame.timestamp_us);
}

std::vector<AudioFrame> AudioRingBuffer::query(uint64_t center_us, uint64_t window_us) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex_);

  uint64_t half = window_us / 2;
  uint64_t lo = (center_us > half) ? center_us - half : 0;
  uint64_t hi = center_us + half;

  std::vector<AudioFrame> result;
  for (const AudioFrame &frame : impl_->frames_)
  {
    if (frame.timestamp_us >= lo && frame.timestamp_us <= hi)
      result.push_back(frame);
  }
  return result;
}

} /* namespace libfreenect2 */
