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

/** @file audio_capture.cpp libusb-backed Kinect v2 mic array capture.
 *
 * Reads the mic array directly via libusb isochronous transfers on the
 * USB Audio Class 1.0 streaming interface (interface 3, alt-setting 1,
 * endpoint 0x85), the same way core libfreenect2 reads video/depth --
 * instead of going through the OS audio stack (WASAPI on Windows).
 *
 * This isn't a style preference: it's a hard requirement. libfreenect2's own
 * Freenect2Impl::openDevice() unconditionally calls libusb_reset_device()
 * before opening the video device (src/libfreenect2.cpp). That's a real
 * USB *port-level* reset -- it briefly drops and re-enumerates the whole
 * physical composite device, including the mic array's child interface.
 * A WASAPI/RtAudio-based capture session never recovers from that (confirmed
 * empirically: PnP status on the affected sub-devices flips OK -> Unknown
 * within under a second of the reset and does not self-heal on any
 * reasonable timeline). Reading the mic array the same way core does --
 * raw libusb isochronous transfers, through UsbDk so this doesn't fight the
 * Kinect SDK's own driver binding either -- sidesteps the problem entirely:
 * both the video and audio interfaces are claimed in the same libusb
 * session, after the same one-time reset, and neither depends on an OS
 * audio driver that reset can knock out from under it.
 *
 * The one-time reset must still happen exactly once, whichever of core
 * (video) or this backend (audio) opens the physical device first --
 * a second, redundant reset would break whichever session is already
 * active, the same failure mode as the WASAPI case above. Order does NOT
 * matter: this backend coordinates with core via
 * device_reset_coordination.h (same-process only -- a std::mutex/
 * condition_variable in the shared freenect2_logging library both link),
 * so whichever of the two calls libusb_reset_device() first "wins" and the
 * other skips its own reset. See that header for what this does and does
 * not cover.
 */

#include <internal/audio/audio_capture_impl.h>
#include "internal/libfreenect2/logging.h"
#include "internal/libfreenect2/device_reset_coordination.h"

#include <libfreenect2/libfreenect2.hpp>

#include <chrono>
#include <cstring>

#if defined(_WIN32)
#include <winsock.h> /* struct timeval -- deliberately NOT windows.h/winsock2.h, see below */
#else
#include <sys/time.h>
#endif

namespace libfreenect2
{

namespace
{
// USB Audio Class 1.0 streaming interface, confirmed via descriptor probe
// against real hardware this session (not assumed): interface 2 is
// AudioControl, interface 3 is AudioStreaming with two alt-settings (0 =
// zero-bandwidth idle, 1 = active isochronous streaming).
const int kAudioControlInterface = 2;
const int kAudioStreamingInterface = 3;
const int kAudioStreamingAltSetting = 1;
const unsigned char kIsoEndpoint = 0x85;

// From the device's own USB Audio Class Type I FORMAT_TYPE descriptor
// (bDescriptorSubtype 0x02) on interface 3/alt 1, read directly via
// libusb_get_config_descriptor's interface_descriptor.extra bytes:
//   nrChannels=4, subframeSize=4 bytes, bitResolution=32, tSamFreq=16000.
// Standard UAC1 PCM Type I framing: signed integer samples, little-endian,
// interleaved per channel (ch0,ch1,ch2,ch3, ch0,ch1,...).
const int kChannels = 4;
const int kSampleRate = 16000;
const int kBytesPerSample = 4;
const int kMaxPacketSize = 256; // wMaxPacketSize on endpoint 0x85, confirmed via descriptor probe.

const int kPacketsPerTransfer = 8;
const int kNumTransfers = 8;
const unsigned int kEventLoopTimeoutMs = 100; // matches core libfreenect2's own usb::EventLoop.
} // namespace

uint64_t nowMicroseconds()
{
  using namespace std::chrono;
  return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

AudioFrame::AudioFrame() :
  frame_count(0),
  channels(0),
  sample_rate(0),
  timestamp_us(0)
{
}

AudioCaptureImpl::AudioCaptureImpl() :
  owns_usb_session_(true),
  ctx_(nullptr),
  handle_(nullptr),
  shutdown_(false),
  pending_cancellations_(0),
  listener_(nullptr),
  deviceFound_(false),
  channels_(kChannels),
  sample_rate_(kSampleRate)
{
  if (libusb_init(&ctx_) != LIBUSB_SUCCESS)
  {
    ctx_ = nullptr;
    return;
  }

  libusb_device *device = nullptr;
  deviceFound_ = findDevice(&device);
  if (device != nullptr)
    libusb_unref_device(device);
}

AudioCaptureImpl::AudioCaptureImpl(Freenect2Device *device) :
  owns_usb_session_(false),
  ctx_(nullptr),
  handle_(nullptr),
  shutdown_(false),
  pending_cancellations_(0),
  listener_(nullptr),
  deviceFound_(false),
  channels_(kChannels),
  sample_rate_(kSampleRate)
{
  ctx_ = static_cast<libusb_context *>(device->getUsbContext());
  handle_ = static_cast<libusb_device_handle *>(device->getUsbDeviceHandle());
  deviceFound_ = (ctx_ != nullptr && handle_ != nullptr);
}

AudioCaptureImpl::~AudioCaptureImpl()
{
  stop();
  if (owns_usb_session_ && ctx_ != nullptr)
    libusb_exit(ctx_);
}

bool AudioCaptureImpl::findDevice(libusb_device **out_device) const
{
  *out_device = nullptr;

  libusb_device **list;
  ssize_t count = libusb_get_device_list(ctx_, &list);
  if (count < 0)
    return false;

  bool found = false;
  for (ssize_t i = 0; i < count; i++)
  {
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(list[i], &desc) != LIBUSB_SUCCESS)
      continue;

    if (desc.idVendor == Freenect2Device::VendorId &&
        (desc.idProduct == Freenect2Device::ProductId || desc.idProduct == Freenect2Device::ProductIdPreview))
    {
      *out_device = list[i];
      libusb_ref_device(*out_device);
      found = true;
      break;
    }
  }

  libusb_free_device_list(list, 1);
  return found;
}

bool AudioCaptureImpl::isAvailable() const
{
  return deviceFound_;
}

bool AudioCaptureImpl::start(AudioFrameListener *listener)
{
  if (!deviceFound_)
  {
    LOG_ERROR("Kinect v2 mic array not found");
    return false;
  }

  if (!transfers_.empty())
  {
    LOG_ERROR("audio capture already started");
    return false;
  }

  if (owns_usb_session_)
  {
    // Same option core libfreenect2 sets for the video device
    // (libfreenect2.cpp) -- without it, libusb_open() fails outright
    // against the Kinect SDK's own driver binding on this interface.
    (void)libusb_set_option(ctx_, LIBUSB_OPTION_USE_USBDK);

    libusb_device *device = nullptr;
    if (!findDevice(&device))
    {
      LOG_ERROR("Kinect v2 mic array disappeared before audio capture could start");
      return false;
    }

    int r = libusb_open(device, &handle_);
    libusb_unref_device(device);
    if (r != LIBUSB_SUCCESS)
    {
      LOG_ERROR("failed to open Kinect v2 audio interface: {}", libusb_error_name(r));
      handle_ = nullptr;
      return false;
    }

    // Whichever of core libfreenect2 (video) or this backend opens the
    // device first in this process is responsible for the one-time
    // device-wide USB reset -- see device_reset_coordination.h. If core
    // already did it (or is doing it right now), this returns false
    // immediately after the reset finishes, and we deliberately do NOT
    // reset again ourselves.
    if (claimDeviceResetResponsibility())
    {
      r = libusb_reset_device(handle_);
      deviceResetComplete();
      if (r != LIBUSB_SUCCESS)
      {
        LOG_ERROR("failed to reset Kinect v2 for audio capture: {}", libusb_error_name(r));
        libusb_close(handle_);
        handle_ = nullptr;
        return false;
      }
    }
  }
  // else: handle_/ctx_ were borrowed from an already-open Freenect2Device
  // (constructed via AudioCaptureImpl(Freenect2Device*)) -- already open,
  // already reset, never reset again here (see device_reset_coordination.h
  // and the class-level comment on the AudioCapture(Freenect2Device*)
  // constructor for why a second reset would break the active video
  // session).

  int r = libusb_claim_interface(handle_, kAudioControlInterface);
  if (r != LIBUSB_SUCCESS)
  {
    LOG_ERROR("failed to claim audio control interface: {} (is the video device already open? "
              "this backend relies on the one-time reset that Freenect2::openDevice() performs)",
              libusb_error_name(r));
    if (owns_usb_session_)
    {
      libusb_close(handle_);
      handle_ = nullptr;
    }
    return false;
  }

  r = libusb_claim_interface(handle_, kAudioStreamingInterface);
  if (r != LIBUSB_SUCCESS)
  {
    LOG_ERROR("failed to claim audio streaming interface: {}", libusb_error_name(r));
    libusb_release_interface(handle_, kAudioControlInterface);
    if (owns_usb_session_)
    {
      libusb_close(handle_);
      handle_ = nullptr;
    }
    return false;
  }

  r = libusb_set_interface_alt_setting(handle_, kAudioStreamingInterface, kAudioStreamingAltSetting);
  if (r != LIBUSB_SUCCESS)
  {
    LOG_ERROR("failed to switch audio streaming interface to isochronous alt-setting: {}", libusb_error_name(r));
    libusb_release_interface(handle_, kAudioStreamingInterface);
    libusb_release_interface(handle_, kAudioControlInterface);
    if (owns_usb_session_)
    {
      libusb_close(handle_);
      handle_ = nullptr;
    }
    return false;
  }

  listener_ = listener;
  channels_ = kChannels;
  sample_rate_ = kSampleRate;
  partial_frame_.clear();
  shutdown_ = false;
  pending_cancellations_ = 0;

  int buf_len = kMaxPacketSize * kPacketsPerTransfer;
  transfers_.reserve(kNumTransfers);
  transfer_buffers_.reserve(kNumTransfers);
  for (int i = 0; i < kNumTransfers; i++)
  {
    unsigned char *buffer = new unsigned char[buf_len];
    struct libusb_transfer *transfer = libusb_alloc_transfer(kPacketsPerTransfer);
    libusb_fill_iso_transfer(transfer, handle_, kIsoEndpoint, buffer, buf_len,
                              kPacketsPerTransfer, &AudioCaptureImpl::transferCallback, this, 1000);
    libusb_set_iso_packet_lengths(transfer, kMaxPacketSize);

    transfer_buffers_.push_back(buffer);
    transfers_.push_back(transfer);

    r = libusb_submit_transfer(transfer);
    if (r != LIBUSB_SUCCESS)
      LOG_ERROR("failed to submit audio isochronous transfer {}: {}", i, libusb_error_name(r));
  }

  event_thread_ = std::thread(&AudioCaptureImpl::eventLoopMain, this);

  LOG_INFO("started audio capture: {} channels @ {} Hz (libusb isochronous)", channels_, sample_rate_);
  return true;
}

void AudioCaptureImpl::stop()
{
  if (transfers_.empty())
    return;

  shutdown_ = true;

  // pending_cancellations_ must be incremented here, once per transfer we
  // actually ask libusb to cancel -- eventLoopMain()'s wait loop below
  // (via event_thread_.join()) uses this count to know when every
  // in-flight transfer has reached a terminal state. Before this fix, this
  // loop never touched pending_cancellations_ at all (only
  // handleCompletedTransfer()'s shutdown branch did), so the count could
  // still be 0 immediately after issuing kNumTransfers cancel requests --
  // eventLoopMain() would then see "!shutdown_ || pending_cancellations_ >
  // 0" as false and exit immediately, letting the code below free every
  // transfer/buffer and (for an owned USB session) close the handle while
  // some of those just-cancelled transfers were still actually in flight
  // with the driver. A late completion for one of them then wrote into
  // already-freed memory -- a real, reliably reproducible use-after-free
  // (confirmed via a minimal AudioCapture start()-then-immediate-stop()
  // repro, both from the plain C++ AudioCapture.cpp example and via
  // pyfreenect2, crashing every time). Only count transfers libusb
  // actually accepted for cancellation (return value LIBUSB_SUCCESS) --
  // one that's already completed/never left the submit queue won't get a
  // callback at all, and waiting on it would hang instead of crash.
  for (auto *transfer : transfers_)
  {
    if (libusb_cancel_transfer(transfer) == LIBUSB_SUCCESS)
      pending_cancellations_++;
  }

  if (event_thread_.joinable())
    event_thread_.join();

  for (size_t i = 0; i < transfers_.size(); i++)
  {
    libusb_free_transfer(transfers_[i]);
    delete[] transfer_buffers_[i];
  }
  transfers_.clear();
  transfer_buffers_.clear();

  libusb_release_interface(handle_, kAudioStreamingInterface);
  libusb_release_interface(handle_, kAudioControlInterface);

  if (owns_usb_session_)
  {
    libusb_close(handle_);
    handle_ = nullptr;
  }
  // else: handle_/ctx_ belong to the Freenect2Device we borrowed them from
  // -- only release the interfaces we claimed, never close/exit someone
  // else's still-in-use USB session.

  listener_ = nullptr;
}

void AudioCaptureImpl::eventLoopMain()
{
  // Bounded timeout throughout -- an unbounded libusb_handle_events() call
  // can block forever if UsbDk never delivers a completion/cancellation
  // event for a given transfer, which happened during development and
  // required a hard process kill to recover from (see
  // feedback_clean_kinect_shutdown memory: that itself risked wedging
  // UsbDk's redirector session until a full reboot). Never remove this
  // timeout in favor of a blocking wait.
  while (!shutdown_ || pending_cancellations_.load() > 0)
  {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = kEventLoopTimeoutMs * 1000;
    libusb_handle_events_timeout(ctx_, &tv);

    if (shutdown_ && pending_cancellations_.load() <= 0)
      break;
  }
}

void LIBUSB_CALL AudioCaptureImpl::transferCallback(struct libusb_transfer *transfer)
{
  static_cast<AudioCaptureImpl *>(transfer->user_data)->handleCompletedTransfer(transfer);
}

void AudioCaptureImpl::handleCompletedTransfer(struct libusb_transfer *transfer)
{
  // Once shutdown_ is set, any status this callback arrives with --
  // LIBUSB_TRANSFER_CANCELLED because stop()'s cancel request landed in
  // time, LIBUSB_TRANSFER_COMPLETED because the transfer happened to
  // finish right around the same time instead, or any other terminal
  // status -- means this transfer is now done and will not fire again.
  // That's exactly what stop()'s pending_cancellations_ wait is tracking,
  // so count it here unconditionally. Do not resubmit (we're shutting
  // down) and do not call libusb_cancel_transfer() again here: there is
  // nothing still in flight to cancel from inside a transfer's own
  // terminal callback, and doing so previously left pending_cancellations_
  // incremented with no matching decrement ever coming (this callback
  // fires exactly once per submitted transfer), which would have hung
  // eventLoopMain()'s wait forever once the use-after-free above was fixed
  // on its own.
  if (shutdown_)
  {
    pending_cancellations_--;
    return;
  }

  for (int p = 0; p < transfer->num_iso_packets; p++)
  {
    const struct libusb_iso_packet_descriptor *pkt = &transfer->iso_packet_desc[p];
    if (pkt->status == LIBUSB_TRANSFER_COMPLETED && pkt->actual_length > 0)
    {
      unsigned char *data = libusb_get_iso_packet_buffer_simple(transfer, p);
      deliverSamples(data, pkt->actual_length);
    }
  }

  int r = libusb_submit_transfer(transfer);
  if (r != LIBUSB_SUCCESS)
    LOG_WARNING("failed to resubmit audio isochronous transfer: {}", libusb_error_name(r));
}

void AudioCaptureImpl::deliverSamples(const unsigned char *data, int length)
{
  if (listener_ == nullptr)
    return;

  // Raw payload is signed 32-bit little-endian PCM, interleaved per channel
  // (ch0,ch1,ch2,ch3, ch0,ch1,...) -- confirmed via the device's own USB
  // Audio Class Type I FORMAT_TYPE descriptor (nrChannels=4, subframeSize=4,
  // bitResolution=32), not assumed from byte-volume math alone.
  int num_samples = length / kBytesPerSample;
  partial_frame_.reserve(partial_frame_.size() + num_samples);

  for (int i = 0; i < num_samples; i++)
  {
    int32_t raw;
    std::memcpy(&raw, data + i * kBytesPerSample, sizeof(raw));
    partial_frame_.push_back(static_cast<float>(raw) / 2147483648.0f);
  }

  // Only deliver on whole multi-channel sample-frame boundaries -- isochronous
  // packets don't align to channel-group boundaries in general.
  size_t whole_frames = partial_frame_.size() / channels_;
  if (whole_frames == 0)
    return;

  size_t whole_samples = whole_frames * channels_;

  auto *frame = new AudioFrame();
  frame->channels = channels_;
  frame->sample_rate = sample_rate_;
  frame->frame_count = whole_frames;
  frame->timestamp_us = nowMicroseconds();
  frame->samples.assign(partial_frame_.begin(), partial_frame_.begin() + static_cast<long>(whole_samples));

  partial_frame_.erase(partial_frame_.begin(), partial_frame_.begin() + static_cast<long>(whole_samples));

  if (!listener_->onNewAudioFrame(frame))
    delete frame;
}

AudioCapture::AudioCapture() : impl_(std::make_unique<AudioCaptureImpl>())
{
}

AudioCapture::AudioCapture(Freenect2Device *device) : impl_(std::make_unique<AudioCaptureImpl>(device))
{
}

AudioCapture::~AudioCapture() noexcept = default;

bool AudioCapture::isAvailable() const
{
  return impl_->isAvailable();
}

bool AudioCapture::start(AudioFrameListener *listener)
{
  return impl_->start(listener);
}

void AudioCapture::stop()
{
  impl_->stop();
}

} /* namespace libfreenect2 */
