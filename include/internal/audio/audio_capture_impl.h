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

/** @file audio_capture_impl.h libusb-backed implementation of AudioCapture.
 *
 * Reads the mic array's isochronous audio endpoint directly via libusb
 * (through UsbDk), the same way core libfreenect2 reads video/depth, instead
 * of going through the OS audio stack (WASAPI/usbaudio.sys) -- see
 * audio_capture.cpp for why: a plain OS-audio-API backend cannot survive the
 * device-wide USB reset libfreenect2 performs when opening the video device.
 */

#ifndef AUDIO_CAPTURE_IMPL_H_
#define AUDIO_CAPTURE_IMPL_H_

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include <libfreenect2/audio_capture.hpp>

namespace libfreenect2
{

class AudioCaptureImpl
{
public:
  // Opens its own independent USB session (see AudioCapture()).
  AudioCaptureImpl();
  // Reuses an already-open Freenect2Device's USB session (see
  // AudioCapture(Freenect2Device*)). Does not own ctx/handle -- never
  // closes or resets them, only claims/releases its own interfaces.
  explicit AudioCaptureImpl(Freenect2Device *device);
  ~AudioCaptureImpl();

  bool isAvailable() const;
  bool start(AudioFrameListener *listener);
  void stop();

private:
  static void LIBUSB_CALL transferCallback(struct libusb_transfer *transfer);
  void handleCompletedTransfer(struct libusb_transfer *transfer);
  void deliverSamples(const unsigned char *data, int length);
  void eventLoopMain();

  bool findDevice(libusb_device **out_device) const;

  bool owns_usb_session_;
  libusb_context *ctx_;
  libusb_device_handle *handle_;
  std::vector<struct libusb_transfer *> transfers_;
  std::vector<unsigned char *> transfer_buffers_;

  std::thread event_thread_;
  std::atomic<bool> shutdown_;
  std::atomic<int> pending_cancellations_;

  AudioFrameListener *listener_;
  std::vector<float> partial_frame_;

  bool deviceFound_;
  int channels_;
  int sample_rate_;
};

} /* namespace libfreenect2 */
#endif /* AUDIO_CAPTURE_IMPL_H_ */
