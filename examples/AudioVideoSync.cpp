/**
 * @file AudioVideoSync.cpp
 * @brief Demonstrates correlating captured audio with depth/RGB frame arrival.
 *
 * Opens the Kinect's Color+Depth stream and the mic array simultaneously.
 * Each depth/RGB frame's arrival is tagged with libfreenect2::nowMicroseconds()
 * — the same monotonic clock AudioFrame::timestamp_us uses — and used to
 * query an AudioRingBuffer for the audio captured around that same instant.
 *
 * Phase 2 of the audio work: proves the correlation mechanism end-to-end.
 * Not yet wired into PCLPointCloud.cpp or doing anything with the audio
 * beyond reporting it (that's phases 3/4).
 */

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/audio_capture.hpp>
#include <libfreenect2/audio_ring_buffer.hpp>

namespace
{
bool sync_shutdown = false;

void sigint_handler(int)
{
  sync_shutdown = true;
}

class RingBufferFeeder : public libfreenect2::AudioFrameListener
{
public:
  explicit RingBufferFeeder(libfreenect2::AudioRingBuffer &ring) : ring_(ring) {}

  bool onNewAudioFrame(libfreenect2::AudioFrame *frame) override
  {
    ring_.push(*frame);
    return false; // AudioCapture deletes the original; we kept our own copy.
  }

private:
  libfreenect2::AudioRingBuffer &ring_;
};
} // namespace

int main()
{
  // Simultaneous audio+video capture. Earlier attempts at this went through
  // an OS audio API (WASAPI via RtAudio) for the mic array, which cannot
  // survive the device-wide USB reset libfreenect2 performs when opening
  // the video device -- that reset is a real port-level action, so it
  // drops and re-enumerates the whole physical composite device including
  // the mic array's child interface, and the OS-level capture session
  // never recovers. AudioCapture(device) instead reuses the video device's
  // already-open libusb session directly (see audio_capture.hpp), the same
  // way core libfreenect2 itself reads video/depth -- no separate OS audio
  // driver involved, so there's nothing for the reset to knock out.
  libfreenect2::Freenect2 freenect2;

  if (freenect2.enumerateDevices() == 0)
  {
    std::cerr << "No Kinect v2 device found.\n";
    return 1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  libfreenect2::Freenect2Device *device = freenect2.openDevice(serial);
  if (!device)
  {
    std::cerr << "Failed to open device with serial " << serial << ".\n";
    return 1;
  }

  libfreenect2::SyncMultiFrameListener listener(
      libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
  device->setColorFrameListener(&listener);
  device->setIrAndDepthFrameListener(&listener);

  if (!device->start())
  {
    std::cerr << "Failed to start device.\n";
    return 1;
  }

  libfreenect2::AudioCapture audio(device);
  if (!audio.isAvailable())
  {
    std::cerr << "No Kinect v2 microphone array found — continuing video-only.\n";
  }

  libfreenect2::AudioRingBuffer ring(5000000);
  RingBufferFeeder feeder(ring);
  bool audio_running = audio.isAvailable() && audio.start(&feeder);

  std::cout << "Streaming depth/RGB" << (audio_running ? " + audio" : "")
            << ". Ctrl-C to stop.\n";

  signal(SIGINT, sigint_handler);

  const uint64_t kCorrelationWindowUs = 33000; // ~one frame period at 30fps

  while (!sync_shutdown)
  {
    libfreenect2::FrameMap frames;
    if (!listener.waitForNewFrame(frames, 1000))
      continue;

    uint64_t arrival_us = libfreenect2::nowMicroseconds();

    if (audio_running)
    {
      std::vector<libfreenect2::AudioFrame> matched = ring.query(arrival_us, kCorrelationWindowUs);

      float peak = 0.0f;
      size_t total_samples = 0;
      for (const auto &af : matched)
      {
        total_samples += af.samples.size();
        for (float s : af.samples)
          peak = std::max(peak, std::fabs(s));
      }

      std::printf("frame t=%llu us  matched=%zu audio buffers (%zu samples)  peak=%.4f\n",
                  static_cast<unsigned long long>(arrival_us), matched.size(), total_samples, peak);
      std::fflush(stdout);
    }

    listener.release(frames);
  }

  if (audio_running)
    audio.stop();

  (void)device->stop();
  (void)device->close();

  return 0;
}
