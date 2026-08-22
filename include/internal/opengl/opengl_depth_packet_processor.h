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

/** @file opengl_depth_packet_processor.h Depth packet processor using OpenGL. */

#ifndef OPENGL_DEPTH_PACKET_PROCESSOR_H_
#define OPENGL_DEPTH_PACKET_PROCESSOR_H_

#include <internal/libfreenect2/depth_packet_processor.h>
#include <freenect2_opengl/export.h>

namespace libfreenect2
{

class OpenGLDepthPacketProcessorImpl;

/** Depth packet processor using OpenGL. */
class FREENECT2_OPENGL_EXPORT OpenGLDepthPacketProcessor : public DepthPacketProcessor
{
public:
  OpenGLDepthPacketProcessor(void *parent_opengl_context_ptr, bool debug);
  ~OpenGLDepthPacketProcessor() noexcept override;
  void setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config) override;

  void loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length) override;

  void loadXZTables(const float *xtable, const float *ztable) override;
  void loadLookupTable(const short *lut) override;

  std::string_view name() override { return "OpenGL"; }
  void process(const DepthPacket &packet) override;
private:
  std::unique_ptr<OpenGLDepthPacketProcessorImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* OPENGL_DEPTH_PACKET_PROCESSOR_H_ */
