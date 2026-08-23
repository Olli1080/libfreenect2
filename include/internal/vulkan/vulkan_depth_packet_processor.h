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

/** @file vulkan_depth_packet_processor.h Depth packet processor using Vulkan.
 *
 * Phase 1 (see the "Add a Vulkan depth-processing backend" plan): only
 * processPixelStage1 is implemented so the core Vulkan plumbing (buffer
 * upload, descriptor sets, push constants, dispatch, barrier, readback) can
 * be proven correct against the OpenCL backend's own stage-1 output before
 * the remaining 3 stages are ported on top of it. process() currently
 * populates only the intermediate a/b/n/ir buffers (exposed for
 * verification) and does not yet produce a final depth Frame.
 */

#ifndef VULKAN_DEPTH_PACKET_PROCESSOR_H_
#define VULKAN_DEPTH_PACKET_PROCESSOR_H_

#include <internal/libfreenect2/depth_packet_processor.h>
#include <freenect2_vulkan/export.h>

#include <vector>

namespace libfreenect2
{

class VulkanDepthPacketProcessorImpl;

/** Depth packet processor using Vulkan. Phase 1: stage 1 only, see the
 * file-level comment above. */
class FREENECT2_VULKAN_EXPORT VulkanDepthPacketProcessor : public DepthPacketProcessor
{
public:
  explicit VulkanDepthPacketProcessor(int deviceId = -1);
  ~VulkanDepthPacketProcessor() noexcept override;
  void setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config) override;

  void loadP0TablesFromCommandResponse(unsigned char *buffer, size_t buffer_length) override;

  void loadXZTables(const float *xtable, const float *ztable) override;
  void loadLookupTable(const short *lut) override;

  bool good() const override;
  std::string_view name() override { return "Vulkan"; }

  void process(const DepthPacket &packet) override;

  /** Phase-1-only verification hooks: intermediate stage-1 output for the
   * most recently processed packet, one entry per pixel (TABLE_SIZE),
   * interleaved xyz per pixel for a/b/n (matching the OpenCL kernel's
   * float3 layout). Removed once the full pipeline (Phase 2/3) lands and
   * process() produces a real depth Frame instead. */
  const std::vector<float> &debugStage1A() const;
  const std::vector<float> &debugStage1B() const;
  const std::vector<float> &debugStage1N() const;
  const std::vector<float> &debugStage1Ir() const;

protected:
  Allocator *getAllocator() override;

private:
  std::unique_ptr<VulkanDepthPacketProcessorImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* VULKAN_DEPTH_PACKET_PROCESSOR_H_ */
