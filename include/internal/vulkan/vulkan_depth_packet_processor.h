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
 * Phase 2 (see the "Add a Vulkan depth-processing backend" plan): all 4
 * depth-decode stages (processPixelStage1, filterPixelStage1,
 * processPixelStage2, filterPixelStage2) are ported and chained with
 * barriers in a single command buffer submission. process() produces a
 * real depth + IR Frame and notifies listener_, same as the OpenCL/CPU
 * backends. The bilateral (stage 1) and edge-aware (stage 2) filters are
 * currently always applied -- the EnableBilateralFilter/EnableEdgeAwareFilter
 * config toggles other backends respect aren't wired up yet (Phase 3).
 */

#ifndef VULKAN_DEPTH_PACKET_PROCESSOR_H_
#define VULKAN_DEPTH_PACKET_PROCESSOR_H_

#include <internal/libfreenect2/depth_packet_processor.h>
#include <freenect2_vulkan/export.h>

#include <cstdint>
#include <vector>

namespace libfreenect2
{

class VulkanDepthPacketProcessorImpl;

/** Depth packet processor using Vulkan. Phase 2: all 4 stages, see the
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

  /** Verification hooks: intermediate output of each stage for the most
   * recently processed packet, one entry per pixel (TABLE_SIZE). a/b/n and
   * their filtered counterparts are interleaved xyz per pixel (matching the
   * OpenCL kernel's float3 layout); depth/ir_sum/filtered/ir are one float
   * per pixel; max_edge_test is one 0-or-1 value per pixel. Used to diff
   * against OpenCL's equivalent buffers on real captured hardware data. */
  const std::vector<float> &debugStage1A() const;
  const std::vector<float> &debugStage1B() const;
  const std::vector<float> &debugStage1N() const;
  const std::vector<float> &debugStage1Ir() const;
  const std::vector<float> &debugFilteredA() const;
  const std::vector<float> &debugFilteredB() const;
  const std::vector<uint32_t> &debugMaxEdgeTest() const;
  const std::vector<float> &debugDepth() const;
  const std::vector<float> &debugIrSum() const;
  const std::vector<float> &debugFiltered() const;

protected:
  Allocator *getAllocator() override;

private:
  std::unique_ptr<VulkanDepthPacketProcessorImpl> impl_;
};

} /* namespace libfreenect2 */
#endif /* VULKAN_DEPTH_PACKET_PROCESSOR_H_ */
