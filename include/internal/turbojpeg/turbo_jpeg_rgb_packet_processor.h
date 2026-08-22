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

/** @file turbo_jpeg_rgb_packet_processor.h RGB packet processor using TurboJPEG. */

#ifndef TURBO_JPEG_RGB_PACKET_PROCESSOR_H_
#define TURBO_JPEG_RGB_PACKET_PROCESSOR_H_

#include <internal/libfreenect2/rgb_packet_processor.h>
#include <freenect2_turbojpeg/export.h>

namespace libfreenect2
{

class TurboJpegRgbPacketProcessorImpl;

/** Processor to decode JPEG to image, using TurboJpeg. */
class FREENECT2_TURBOJPEG_EXPORT TurboJpegRgbPacketProcessor : public RgbPacketProcessor
{
public:
  TurboJpegRgbPacketProcessor();
  ~TurboJpegRgbPacketProcessor() noexcept override;
  void process(const libfreenect2::RgbPacket &packet) override;
  std::string_view name() override { return "TurboJPEG"; }
private:
  std::unique_ptr<TurboJpegRgbPacketProcessorImpl> impl_; ///< Decoder implementation.
};

} /* namespace libfreenect2 */
#endif /* TURBO_JPEG_RGB_PACKET_PROCESSOR_H_ */
