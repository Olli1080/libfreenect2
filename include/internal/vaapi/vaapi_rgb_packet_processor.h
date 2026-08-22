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

/** @file vaapi_rgb_packet_processor.h RGB packet processor using Intel VA-API (deprecated). */

#ifndef VAAPI_RGB_PACKET_PROCESSOR_H_
#define VAAPI_RGB_PACKET_PROCESSOR_H_

#include <internal/libfreenect2/rgb_packet_processor.h>
#include <freenect2_vaapi/export.h>

namespace libfreenect2
{

class VaapiRgbPacketProcessorImpl;

class FREENECT2_VAAPI_EXPORT VaapiRgbPacketProcessor : public RgbPacketProcessor
{
public:
  VaapiRgbPacketProcessor();
  ~VaapiRgbPacketProcessor() noexcept override;
  bool good() const override;
  std::string_view name() override { return "VAAPI"; }
  void process(const libfreenect2::RgbPacket &packet) override;
protected:
  Allocator *getAllocator() override;
private:
  std::unique_ptr<VaapiRgbPacketProcessorImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* VAAPI_RGB_PACKET_PROCESSOR_H_ */
