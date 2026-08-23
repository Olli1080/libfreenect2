/**
 * @file AudioCapture.cpp
 * @brief Minimal Kinect v2 microphone array capture example.
 *
 * Prints a timestamp and rough peak amplitude for each captured audio
 * buffer until Ctrl-C. Proves freenect2_audio end-to-end; not integrated
 * with the depth/RGB pipeline (see docs/plans for later phases).
 */

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <iostream>

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <ctime>
#endif

#include <libfreenect2/audio_capture.hpp>

namespace
{
bool audio_shutdown = false;

void sigint_handler(int)
{
  audio_shutdown = true;
}

class PrintingListener : public libfreenect2::AudioFrameListener
{
public:
  bool onNewAudioFrame(libfreenect2::AudioFrame *frame) override
  {
    float peak = 0.0f;
    for (float s : frame->samples)
      peak = std::max(peak, std::fabs(s));

    std::printf("t=%llu us  frames=%zu  ch=%d  rate=%d  peak=%.4f\n",
                static_cast<unsigned long long>(frame->timestamp_us),
                frame->frame_count, frame->channels, frame->sample_rate, peak);
    std::fflush(stdout);

    delete frame;
    return true;
  }
};
} // namespace

int main()
{
  libfreenect2::AudioCapture capture;

  if (!capture.isAvailable())
  {
    std::cerr << "No Kinect v2 microphone array found.\n";
    return 1;
  }

  PrintingListener listener;
  if (!capture.start(&listener))
  {
    std::cerr << "Failed to start audio capture.\n";
    return 1;
  }

  std::cout << "Capturing from Kinect v2 mic array. Ctrl-C to stop.\n";

  signal(SIGINT, sigint_handler);
  while (!audio_shutdown)
  {
#if defined(_WIN32)
    Sleep(100);
#else
    struct timespec ts { 0, 100 * 1000 * 1000 };
    nanosleep(&ts, nullptr);
#endif
  }

  capture.stop();
  return 0;
}
