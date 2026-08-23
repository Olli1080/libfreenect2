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

/** @file audio_ring_buffer.hpp Timestamp-queryable rolling window of audio,
 * for correlating captured AudioFrames with depth/RGB Frame arrival times. */

#ifndef LIBFREENECT2_AUDIO_RING_BUFFER_HPP_
#define LIBFREENECT2_AUDIO_RING_BUFFER_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <freenect2_audio/export.h>
#include <libfreenect2/audio_capture.hpp>

namespace libfreenect2
{

/** Thread-safe rolling window of recently-captured AudioFrames, queryable by
 * timestamp. Feed it from an AudioFrameListener (any thread); query it from
 * any other thread (e.g. your depth/RGB frame loop) to correlate audio with
 * a Frame's arrival time — tag that arrival with nowMicroseconds() and pass
 * it to query(). @ingroup audio */
class FREENECT2_AUDIO_EXPORT AudioRingBuffer
{
public:
  /** retention_us: how much audio history to keep before older frames are
   * dropped. Must be at least as large as the largest window you intend to
   * query() with. */
  explicit AudioRingBuffer(uint64_t retention_us = 5000000);
  ~AudioRingBuffer();

  /** Takes ownership of frame (it is copied internally; caller may delete
   * or discard frame afterwards). Typically called from
   * AudioFrameListener::onNewAudioFrame(). */
  void push(const AudioFrame &frame);

  /** Returns copies of all retained frames whose timestamp_us falls within
   * [center_us - window_us/2, center_us + window_us/2], oldest first. */
  std::vector<AudioFrame> query(uint64_t center_us, uint64_t window_us) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_AUDIO_RING_BUFFER_HPP_ */
