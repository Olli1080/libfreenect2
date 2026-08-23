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

/** @file opencl_depth_packet_processor.h Depth packet processors using OpenCL. */

#ifndef OPENCL_DEPTH_PACKET_PROCESSOR_H_
#define OPENCL_DEPTH_PACKET_PROCESSOR_H_

#include <internal/libfreenect2/depth_packet_processor.h>
#include <freenect2_opencl/export.h>

#include <vector>

namespace libfreenect2
{

class OpenCLDepthPacketProcessorImpl;

/** Depth packet processor using OpenCL. */
class FREENECT2_OPENCL_EXPORT OpenCLDepthPacketProcessor : public DepthPacketProcessor
{
public:
  OpenCLDepthPacketProcessor(const int deviceId = -1);
  ~OpenCLDepthPacketProcessor() noexcept override;
  void setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config) override;

  void loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length) override;

  void loadXZTables(const float *xtable, const float *ztable) override;
  void loadLookupTable(const short *lut) override;

  bool good() const override;
  std::string_view name() override { return "OpenCL"; }

  void process(const DepthPacket &packet) override;

  /** Debug-only verification hook for the Vulkan backend port (see the "Add
   * a Vulkan depth-processing backend" plan): captures stage 1's
   * intermediate a/b/n/ir buffers for the most recently processed packet so
   * they can be diffed numerically against VulkanDepthPacketProcessor's
   * equivalent debugStage1*() output. Off by default -- enabling it adds a
   * synchronous buffer readback after stage 1 that normal use doesn't need.
   * a/b/n are float4-per-pixel (xyz = one value per IR frequency layer, w
   * padding), matching cl_float3's host layout; ir is float-per-pixel. */
  void enableStage1DebugCapture(bool enable);
  const std::vector<float> &debugStage1A() const;
  const std::vector<float> &debugStage1B() const;
  const std::vector<float> &debugStage1N() const;
  const std::vector<float> &debugStage1Ir() const;
protected:
  Allocator *getAllocator() override;
private:
  std::unique_ptr<OpenCLDepthPacketProcessorImpl> impl_;
};

/*
 * The class below implement a depth packet processor using the phase unwrapping
 * algorithm described in the paper "Efficient Phase Unwrapping using Kernel
 * Density Estimation", ECCV 2016, Felix Järemo Lawin, Per-Erik Forssen and
 * Hannes Ovren, see http://www.cvl.isy.liu.se/research/datasets/kinect2-dataset/.
 */
class OpenCLKdeDepthPacketProcessorImpl;

/** Depth packet processor using OpenCL. */
class FREENECT2_OPENCL_EXPORT OpenCLKdeDepthPacketProcessor : public DepthPacketProcessor
{
public:
  OpenCLKdeDepthPacketProcessor(const int deviceId = -1);
  ~OpenCLKdeDepthPacketProcessor() noexcept override;
  void setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config) override;

  void loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length) override;

  void loadXZTables(const float *xtable, const float *ztable) override;
  void loadLookupTable(const short *lut) override;

  bool good() const override;
  std::string_view name() override { return "OpenCLKde"; }

  void process(const DepthPacket &packet) override;
protected:
  Allocator *getAllocator() override;
private:
  std::unique_ptr<OpenCLKdeDepthPacketProcessorImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* OPENCL_DEPTH_PACKET_PROCESSOR_H_ */
