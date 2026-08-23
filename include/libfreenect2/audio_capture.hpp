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

/** @file audio_capture.hpp Kinect v2 microphone array capture.
 *
 * Not part of libfreenect2 core: the mic array is a standard OS audio
 * capture endpoint (confirmed: Windows exposes it as a plain WASAPI
 * "Microphone Array (Xbox NUI Sensor)" device, no vendor driver or Kinect
 * USB protocol involved), so this lives in its own freenect2_audio library
 * rather than being linked into freenect2 core, which stays video/depth-only.
 */

#ifndef LIBFREENECT2_AUDIO_CAPTURE_HPP_
#define LIBFREENECT2_AUDIO_CAPTURE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <freenect2_audio/export.h>

namespace libfreenect2
{

class Freenect2Device;

/** Monotonic clock, microseconds — the exact clock AudioFrame::timestamp_us
 * uses. Apps tagging depth/RGB Frame arrival times for correlation (see
 * AudioRingBuffer) should stamp with this function, not their own
 * steady_clock call, so both sides are guaranteed the same clock semantics.
 * @ingroup audio */
FREENECT2_AUDIO_EXPORT uint64_t nowMicroseconds();

/** One buffer's worth of captured audio. @ingroup audio */
class FREENECT2_AUDIO_EXPORT AudioFrame
{
public:
  static const int CHANNELS = 4; ///< Kinect v2 mic array channel count.

  std::vector<float> samples;    ///< Interleaved: ch0,ch1,ch2,ch3, ch0,ch1,...
  size_t frame_count;            ///< Multi-channel sample frames (samples.size() / channels).
  int channels;
  int sample_rate;

  /** Monotonic clock timestamp (microseconds), taken when this buffer was
   * delivered by the capture callback. Comparable to
   * std::chrono::steady_clock::now().time_since_epoch() cast to
   * microseconds — NOT wall-clock time, and NOT the same clock domain as
   * Frame::timestamp (which is the Kinect's own device-side clock). */
  uint64_t timestamp_us;

  AudioFrame();
};

/** Callback interface to receive new audio frames. @ingroup audio
 * You can inherit from AudioFrameListener and define your own listener.
 */
class FREENECT2_AUDIO_EXPORT AudioFrameListener
{
public:
  virtual ~AudioFrameListener() noexcept = default;

  /**
   * Called when a new audio buffer is captured.
   * @param frame Captured audio data.
   * @return true if you want to take ownership of the frame (delete it when
   * done). Returning false means the caller deletes it after this call.
   */
  virtual bool onNewAudioFrame(AudioFrame *frame) = 0;
};

class AudioCaptureImpl;

/** Captures audio from the Kinect v2 microphone array. @ingroup audio */
class FREENECT2_AUDIO_EXPORT AudioCapture
{
public:
  /** Opens its own independent USB session to the mic array. Works
   * standalone (no Freenect2Device needed), but on Windows with the UsbDk
   * backend, opening this way at the same time as an already-open
   * Freenect2Device for the same physical Kinect will fail: UsbDk only
   * permits one open session per device. Use the other constructor when
   * you also have an open Freenect2Device for the same Kinect. */
  AudioCapture();

  /** Reuses an already-open Freenect2Device's USB session instead of
   * opening an independent one -- required for simultaneous audio+video
   * capture (see the class-level note on the other constructor). `device`
   * must already be open (i.e. after a successful Freenect2::openDevice())
   * and must outlive this AudioCapture. */
  explicit AudioCapture(Freenect2Device *device);

  ~AudioCapture() noexcept;

  /** Whether a Kinect v2 mic array capture endpoint was found. */
  bool isAvailable() const;

  /** Starts capture, delivering AudioFrames to listener from the capture
   * backend's own callback thread. Returns false on failure (device not
   * found, stream initialization failure). */
  bool start(AudioFrameListener *listener);

  /** Stops capture. Safe to call even if start() was never called or failed. */
  void stop();

private:
  std::unique_ptr<AudioCaptureImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_AUDIO_CAPTURE_HPP_ */
