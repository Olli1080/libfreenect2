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

/** @file VulkanStage1Verify.cpp Phase 1 verification tool for the Vulkan
 * depth-processing backend (see the "Add a Vulkan depth-processing backend"
 * plan). Captures one real raw depth packet plus calibration tables from an
 * actual Kinect via DumpPacketPipeline, then feeds that exact packet through
 * OpenCLDepthPacketProcessor and VulkanDepthPacketProcessor directly
 * (bypassing the normal device pipeline) and numerically diffs their
 * stage-1 (processPixelStage1) intermediate a/b/n/ir output. This is the
 * plan's explicit Phase 1 completion criterion -- not a permanent example
 * app, just the plumbing-correctness check.
 */

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/frame_listener_impl.h>

#include <internal/libfreenect2/depth_packet_processor.h>
#include <internal/opencl/opencl_depth_packet_processor.h>
#include <internal/vulkan/vulkan_depth_packet_processor.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace libfreenect2;

namespace
{

class NullListener : public FrameListener
{
public:
  bool onNewFrame(Frame::Type, Frame *) override { return false; }
};

void compare(const std::vector<float> &reference, const std::vector<float> &candidate, float tolerance, const char *name)
{
  if (reference.size() != candidate.size())
  {
    std::printf("  [%s] SIZE MISMATCH: OpenCL=%zu Vulkan=%zu -- cannot compare\n", name, reference.size(), candidate.size());
    return;
  }

  float max_abs_diff = 0.0f;
  size_t num_exceeding = 0;
  for (size_t i = 0; i < reference.size(); ++i)
  {
    float diff = std::fabs(reference[i] - candidate[i]);
    max_abs_diff = std::max(max_abs_diff, diff);
    if (diff > tolerance)
      ++num_exceeding;
  }

  double pct = reference.empty() ? 0.0 : 100.0 * static_cast<double>(num_exceeding) / static_cast<double>(reference.size());
  std::printf("  [%s] %zu values, max |diff| = %.6f, %zu exceed tolerance %.4f (%.4f%%)\n",
              name, reference.size(), max_abs_diff, num_exceeding, tolerance, pct);
}

} // namespace

int main()
{
  Freenect2 freenect2;
  int num_devices = freenect2.enumerateDevices();
  if (num_devices == 0)
  {
    std::fprintf(stderr, "no Kinect v2 device found\n");
    return 1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  auto *pipeline = new DumpPacketPipeline();
  Freenect2Device *dev = freenect2.openDevice(serial, pipeline);
  if (!dev)
  {
    std::fprintf(stderr, "failed to open device\n");
    return 1;
  }

  SyncMultiFrameListener sync_listener(Frame::Depth);
  dev->setIrAndDepthFrameListener(&sync_listener);

  if (!dev->start())
  {
    std::fprintf(stderr, "failed to start device\n");
    return 1;
  }

  std::printf("waiting for one raw depth packet...\n");
  FrameMap frames;
  sync_listener.waitForNewFrame(frames);

  Frame *raw_depth = frames[Frame::Depth];
  std::vector<unsigned char> raw_packet(raw_depth->data, raw_depth->data + raw_depth->bytes_per_pixel);

  size_t p0_len = 0, x_len = 0, z_len = 0, lut_len = 0;
  const unsigned char *p0_ptr = pipeline->getDepthP0Tables(&p0_len);
  const float *x_ptr = pipeline->getDepthXTable(&x_len);
  const float *z_ptr = pipeline->getDepthZTable(&z_len);
  const short *lut_ptr = pipeline->getDepthLookupTable(&lut_len);

  // Copy everything out before stop()/close() -- the pipeline (and the
  // buffers backing these pointers) is owned by the device and does not
  // outlive it.
  std::vector<unsigned char> p0_table(p0_ptr, p0_ptr + p0_len);
  std::vector<float> x_table(x_ptr, x_ptr + x_len);
  std::vector<float> z_table(z_ptr, z_ptr + z_len);
  std::vector<short> lut(lut_ptr, lut_ptr + lut_len);

  std::printf("captured raw depth packet: %zu bytes, p0=%zu x=%zu z=%zu lut=%zu\n",
              raw_packet.size(), p0_table.size(), x_table.size(), z_table.size(), lut.size());

  sync_listener.release(frames);
  static_cast<void>(dev->stop());
  static_cast<void>(dev->close());

  NullListener null_listener;

  OpenCLDepthPacketProcessor opencl_proc(-1);
  // setFrameListener is inherited (not overridden) from the internal,
  // non-exported DepthPacketProcessor base -- go through a base-class
  // pointer so this is a virtual (vtable) call instead of a direct symbol
  // reference the linker can't resolve across the DLL boundary.
  static_cast<DepthPacketProcessor *>(&opencl_proc)->setFrameListener(&null_listener);
  opencl_proc.loadP0TablesFromCommandResponse(p0_table.data(), p0_table.size());
  opencl_proc.loadXZTables(x_table.data(), z_table.data());
  opencl_proc.loadLookupTable(lut.data());
  opencl_proc.enableStage1DebugCapture(true);

  if (!opencl_proc.good())
  {
    std::fprintf(stderr, "OpenCLDepthPacketProcessor not good() -- no usable OpenCL device\n");
    return 1;
  }

  VulkanDepthPacketProcessor vulkan_proc(-1);
  static_cast<DepthPacketProcessor *>(&vulkan_proc)->setFrameListener(&null_listener);
  vulkan_proc.loadP0TablesFromCommandResponse(p0_table.data(), p0_table.size());
  vulkan_proc.loadXZTables(x_table.data(), z_table.data());
  vulkan_proc.loadLookupTable(lut.data());

  if (!vulkan_proc.good())
  {
    std::fprintf(stderr, "VulkanDepthPacketProcessor not good() -- no usable Vulkan device\n");
    return 1;
  }

  DepthPacket packet{};
  packet.sequence = 0;
  packet.timestamp = 0;
  packet.buffer = raw_packet.data();
  packet.buffer_length = raw_packet.size();
  packet.memory = nullptr;

  opencl_proc.process(packet);
  vulkan_proc.process(packet);

  std::printf("\nStage 1 numeric comparison (OpenCL reference vs Vulkan port), tolerance chosen\n"
              "loosely relative to raw measurement magnitudes (~10^2-10^4):\n");
  compare(opencl_proc.debugStage1A(), vulkan_proc.debugStage1A(), 0.05f, "a");
  compare(opencl_proc.debugStage1B(), vulkan_proc.debugStage1B(), 0.05f, "b");
  compare(opencl_proc.debugStage1N(), vulkan_proc.debugStage1N(), 0.05f, "n");
  compare(opencl_proc.debugStage1Ir(), vulkan_proc.debugStage1Ir(), 0.05f, "ir");

  return 0;
}
