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

/** @file turbo_jpeg_rgb_packet_processor.cpp JPEG decoder with Turbo Jpeg. */

#include <internal/turbojpeg/turbo_jpeg_rgb_packet_processor.h>
#include <libfreenect2/logging.h>
#include <turbojpeg.h>

#include <memory>

namespace libfreenect2
{

struct TjDeleter
{
  void operator()(tjhandle handle) const
  {
    if(handle != nullptr && tjDestroy(handle) == -1)
    {
      LOG_ERROR("Failed to destroy TurboJPEG decompressor! TurboJPEG error: '{}'", tjGetErrorStr());
    }
  }
};

/** Implementation of the Turbo-Jpeg decoder processor. */
class TurboJpegRgbPacketProcessorImpl: public WithPerfLogging
{
public:

  std::unique_ptr<void, TjDeleter> decompressor;

  std::unique_ptr<Frame> frame;

  TurboJpegRgbPacketProcessorImpl()
  {
    decompressor = std::unique_ptr<void, TjDeleter>(tjInitDecompress());
    if(decompressor == nullptr)
    {
      LOG_ERROR("Failed to initialize TurboJPEG decompressor! TurboJPEG error: '{}'", tjGetErrorStr());
    }

    newFrame();
  }

  ~TurboJpegRgbPacketProcessorImpl()
  {
  }

  void newFrame()
  {
    frame = std::make_unique<Frame>(1920, 1080, tjPixelSize[TJPF_BGRX]);
    frame->format = Frame::BGRX;
  }
};

TurboJpegRgbPacketProcessor::TurboJpegRgbPacketProcessor() :
    impl_(std::make_unique<TurboJpegRgbPacketProcessorImpl>())
{
}

TurboJpegRgbPacketProcessor::~TurboJpegRgbPacketProcessor()
{
}

void TurboJpegRgbPacketProcessor::process(const RgbPacket &packet)
{
  if(impl_->decompressor != nullptr && listener_ != nullptr)
  {
    impl_->startTiming();

    impl_->frame->timestamp = packet.timestamp;
    impl_->frame->sequence = packet.sequence;
    impl_->frame->exposure = packet.exposure;
    impl_->frame->gain = packet.gain;
    impl_->frame->gamma = packet.gamma;

    int r = tjDecompress2(impl_->decompressor.get(), packet.jpeg_buffer, packet.jpeg_buffer_length, impl_->frame->data, 1920, 1920 * tjPixelSize[TJPF_BGRX], 1080, TJPF_BGRX, 0);

    std::string timing = impl_->stopTiming(LOG_SOURCE);
    if (!timing.empty())
      LOG_INFO("{}", timing);

    if(r == 0)
    {
      if(listener_->onNewFrame(Frame::Color, impl_->frame.get()))
      {
        impl_->frame.release();
        impl_->newFrame();
      }
    }
    else
    {
      LOG_ERROR("Failed to decompress rgb image! TurboJPEG error: '{}'", tjGetErrorStr());
    }
  }
}

} /* namespace libfreenect2 */
