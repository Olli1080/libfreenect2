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

/** @file packet_pipeline.cpp Packet pipeline implementation. */

#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/async_packet_processor.h>
#include <libfreenect2/data_callback.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <libfreenect2/depth_packet_stream_parser.h>
#include <libfreenect2/protocol/response.h>

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
#include <internal/opengl/opengl_depth_packet_processor.h>
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
#include <internal/opencl/opencl_depth_packet_processor.h>
#endif
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
#include <internal/cuda/cuda_depth_packet_processor.h>
#include <internal/cuda/cuda_kde_depth_packet_processor.h>
#endif
#ifdef LIBFREENECT2_WITH_VT_SUPPORT
#include <internal/vt/vt_rgb_packet_processor.h>
#endif
#ifdef LIBFREENECT2_WITH_VAAPI_SUPPORT
#include <internal/vaapi/vaapi_rgb_packet_processor.h>
#endif
#ifdef LIBFREENECT2_WITH_TURBOJPEG_SUPPORT
#include <internal/turbojpeg/turbo_jpeg_rgb_packet_processor.h>
#endif

namespace libfreenect2
{

static std::unique_ptr<RgbPacketProcessor> getDefaultRgbPacketProcessor()
{
#if defined(LIBFREENECT2_WITH_VT_SUPPORT)
  return std::make_unique<VTRgbPacketProcessor>();
#elif defined(LIBFREENECT2_WITH_VAAPI_SUPPORT)
  auto vaapi = std::make_unique<VaapiRgbPacketProcessor>();
  if (vaapi->good())
    return vaapi;
  return std::make_unique<TurboJpegRgbPacketProcessor>();
#elif defined(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT)
  return std::make_unique<TurboJpegRgbPacketProcessor>();
#else
  #error No jpeg decoder is enabled
#endif
}

class PacketPipelineComponents
{
public:
  std::unique_ptr<RgbPacketStreamParser> rgb_parser_;
  std::unique_ptr<DepthPacketStreamParser> depth_parser_;

  std::unique_ptr<RgbPacketProcessor> rgb_processor_;
  std::unique_ptr<BaseRgbPacketProcessor> async_rgb_processor_;
  std::unique_ptr<DepthPacketProcessor> depth_processor_;
  std::unique_ptr<BaseDepthPacketProcessor> async_depth_processor_;

  ~PacketPipelineComponents() = default;
  void initialize(std::unique_ptr<RgbPacketProcessor> rgb, std::unique_ptr<DepthPacketProcessor> depth);
};

void PacketPipelineComponents::initialize(std::unique_ptr<RgbPacketProcessor> rgb, std::unique_ptr<DepthPacketProcessor> depth)
{
  rgb_parser_ = std::make_unique<RgbPacketStreamParser>();
  depth_parser_ = std::make_unique<DepthPacketStreamParser>();

  rgb_processor_ = std::move(rgb);
  depth_processor_ = std::move(depth);

  async_rgb_processor_ = std::make_unique<AsyncPacketProcessor<RgbPacket>>(rgb_processor_.get());
  async_depth_processor_ = std::make_unique<AsyncPacketProcessor<DepthPacket>>(depth_processor_.get());

  rgb_parser_->setPacketProcessor(async_rgb_processor_.get());
  depth_parser_->setPacketProcessor(async_depth_processor_.get());
}

PacketPipeline::PacketPipeline(): comp_(std::make_unique<PacketPipelineComponents>()) {}

PacketPipeline::~PacketPipeline() = default;

PacketPipeline::PacketParser *PacketPipeline::getRgbPacketParser() const
{
  return comp_->rgb_parser_.get();
}

PacketPipeline::PacketParser *PacketPipeline::getIrPacketParser() const
{
  return comp_->depth_parser_.get();
}

RgbPacketProcessor *PacketPipeline::getRgbPacketProcessor() const
{
  return comp_->rgb_processor_.get();
}

DepthPacketProcessor *PacketPipeline::getDepthPacketProcessor() const
{
  return comp_->depth_processor_.get();
}

CpuPacketPipeline::CpuPacketPipeline()
{
  comp_->initialize(getDefaultRgbPacketProcessor(), std::make_unique<CpuDepthPacketProcessor>());
}

CpuPacketPipeline::~CpuPacketPipeline() { }

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
OpenGLPacketPipeline::OpenGLPacketPipeline(void *parent_opengl_context, bool debug) : parent_opengl_context_(parent_opengl_context), debug_(debug)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), std::make_unique<OpenGLDepthPacketProcessor>(parent_opengl_context_, debug_));
}

OpenGLPacketPipeline::~OpenGLPacketPipeline() { }
#endif // LIBFREENECT2_WITH_OPENGL_SUPPORT


#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
OpenCLPacketPipeline::OpenCLPacketPipeline(const int deviceId) : deviceId(deviceId)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), std::make_unique<OpenCLDepthPacketProcessor>(deviceId));
}

OpenCLPacketPipeline::~OpenCLPacketPipeline() { }


OpenCLKdePacketPipeline::OpenCLKdePacketPipeline(const int deviceId) : deviceId(deviceId)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), std::make_unique<OpenCLKdeDepthPacketProcessor>(deviceId));
}

OpenCLKdePacketPipeline::~OpenCLKdePacketPipeline() { }
#endif // LIBFREENECT2_WITH_OPENCL_SUPPORT

#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
CudaPacketPipeline::CudaPacketPipeline(const int deviceId) : deviceId(deviceId)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), std::make_unique<CudaDepthPacketProcessor>(deviceId));
}

CudaKdePacketPipeline::~CudaKdePacketPipeline() { }

CudaKdePacketPipeline::CudaKdePacketPipeline(const int deviceId) : deviceId(deviceId)
{
  comp_->initialize(getDefaultRgbPacketProcessor(), std::make_unique<CudaKdeDepthPacketProcessor>(deviceId));
}

CudaPacketPipeline::~CudaPacketPipeline() { }
#endif // LIBFREENECT2_WITH_CUDA_SUPPORT

DumpPacketPipeline::DumpPacketPipeline()
{
  comp_->initialize(std::make_unique<DumpRgbPacketProcessor>(), std::make_unique<DumpDepthPacketProcessor>());
}

DumpPacketPipeline::~DumpPacketPipeline() {}

const unsigned char* DumpPacketPipeline::getDepthP0Tables(size_t* length) {
  *length = sizeof(libfreenect2::protocol::P0TablesResponse);
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getP0Tables();
}

const float* DumpPacketPipeline::getDepthXTable(size_t* length) {
  *length = DepthPacketProcessor::TABLE_SIZE;
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getXTable();
}

const float* DumpPacketPipeline::getDepthZTable(size_t* length) {
  *length = DepthPacketProcessor::TABLE_SIZE;
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getZTable();
}

const short* DumpPacketPipeline::getDepthLookupTable(size_t* length) {
  *length = DepthPacketProcessor::LUT_SIZE;
  return static_cast<DumpDepthPacketProcessor*>(getDepthPacketProcessor())->getLookupTable();
}

} /* namespace libfreenect2 */
