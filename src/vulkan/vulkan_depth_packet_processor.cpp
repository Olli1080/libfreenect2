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

/** @file vulkan_depth_packet_processor.cpp Vulkan depth packet processor,
 * see the header's file-level comment for what's implemented.
 *
 * Every buffer is HOST_VISIBLE|HOST_COHERENT with persistent mapping (no
 * staging buffers, no VMA) -- simplicity over raw throughput, since the
 * primary goal was proving the compute plumbing (buffer upload, descriptor
 * sets, push constants, dispatch, readback) correct first. findMemoryType()
 * does prefer a memory type that's *also* DEVICE_LOCAL when one exists
 * (confirmed present on this session's GPU via vulkaninfo -- Resizable BAR
 * exposes the whole VRAM heap as host-visible too), so on such hardware
 * this already avoids routing every GPU-side buffer access over PCIe to
 * system RAM without needing an explicit staging-buffer upload path.
 */

#include <internal/vulkan/vulkan_depth_packet_processor.h>
#include <internal/libfreenect2/protocol/response.h>
#include <libfreenect2/logging.h>

#define VULKAN_HPP_NO_CONSTRUCTORS
#include <vulkan/vulkan.hpp>

#include <cmath>
#include <cstring>
#include <string_view>

namespace
{
// Deliberately not using libfreenect2::loadResource()/resources.inc.h here:
// that shared mechanism's #include "resources.inc.h" in src/resource.cpp is
// a literal filename that only resolves to a stale, pre-modularization
// combined resource snapshot (last regenerated 2026-08-22, before the
// per-backend resources_<name>.inc.h split existed) -- it still happens to
// contain what OpenCL/OpenGL/core need since those resource names haven't
// changed, but a new resource name (this shader) was never in it and never
// will be without fixing that shared mechanism for every backend at once.
// Out of scope here -- this loads the correctly, freshly regenerated
// resources_vulkan.inc.h directly instead, self-contained to this module.
struct ResourceDescriptor
{
  const char *filename;
  const unsigned char *data;
  size_t length;
};
#include "resources_vulkan.inc.h"

bool loadVulkanResource(std::string_view name, const unsigned char **data, size_t *length)
{
  for (int i = 0; i < resource_descriptors_length; ++i)
  {
    if (name == resource_descriptors[i].filename)
    {
      *data = resource_descriptors[i].data;
      *length = resource_descriptors[i].length;
      return true;
    }
  }
  return false;
}
} // namespace

namespace libfreenect2
{

namespace
{
const int IMAGE_SIZE = 512 * 424;
const float kPi = 3.14159265358979323846f;
// Raw packet: 10 sub-frames of 424 rows * 352 packed-11-bit columns, 2 bytes
// each (matches OpenCL's own buf_packet_size in
// opencl_depth_packet_processor.cpp) -- decodePixelMeasurement() only reads
// subs 0-8, the 10th is present in every real packet but unused by stage 1,
// same as the OpenCL kernel. Verified against a real captured packet's size
// (2,984,960 bytes); an earlier 9-sub-frame assumption here was wrong.
const size_t RAW_DATA_SIZE = 10u * 424u * 352u * sizeof(uint16_t);
} // namespace

struct VulkanBufferResource
{
  vk::UniqueBuffer buffer;
  vk::UniqueDeviceMemory memory;
  void *mapped = nullptr;
  vk::DeviceSize size = 0;
};

// One compute pipeline (shader + descriptor set layout/pool/set + pipeline
// layout with an optional scalars-only push constant range) -- Phase 2 has
// 4 of these (one per depth-decode stage), built by createComputeStage()
// below instead of repeating stage 1's original ~80 lines of Vulkan
// boilerplate 4 times.
//
// descSetAlt is optional, used only by stage2: when EnableBilateralFilter
// is off, stage2 must read directly from the unfiltered a/b buffers
// instead of the bilateral filter's output. Descriptor sets are bound to
// fixed buffers at creation time (unlike OpenCL's kernel args, which get
// rebound on config change), so this is a second, alternately-bound
// descriptor set sharing the same pipeline/layout rather than a mutation
// of descSet -- built by createAltDescriptorSet() below.
struct ComputeStage
{
  vk::UniqueShaderModule shaderModule;
  vk::UniqueDescriptorSetLayout descSetLayout;
  vk::UniquePipelineLayout pipelineLayout;
  vk::UniquePipeline pipeline;
  vk::UniqueDescriptorPool descPool;
  vk::UniqueDescriptorSet descSet;
  vk::UniqueDescriptorPool descPoolAlt;
  vk::UniqueDescriptorSet descSetAlt;
};

class VulkanDepthPacketProcessorImpl
{
public:
  bool deviceInitialized = false;
  bool tablesLoaded = false;
  bool debugCapture = false;
  DepthPacketProcessor::Parameters params;
  DepthPacketProcessor::Config config;

  vk::UniqueInstance instance;
  vk::PhysicalDevice physicalDevice;
  vk::UniqueDevice device;
  vk::Queue queue;
  uint32_t queueFamilyIndex = 0;

  vk::UniqueCommandPool commandPool;
  vk::UniqueCommandBuffer commandBuffer;
  vk::UniqueFence fence;

  ComputeStage stage1, filterStage1, stage2, filterStage2;

  VulkanBufferResource lutBuf, zTableBuf, xTableBuf, p0TableBuf, dataBuf;
  VulkanBufferResource aBuf, bBuf, nBuf, irBuf;
  VulkanBufferResource aFilteredBuf, bFilteredBuf, gaussianBuf, maxEdgeTestBuf;
  VulkanBufferResource depthBuf, irSumBuf, filteredBuf;

  std::unique_ptr<Frame> ir_frame;
  std::unique_ptr<Frame> depth_frame;

  std::vector<float> debugA, debugB, debugN, debugIr;
  std::vector<float> debugFilteredA, debugFilteredB;
  std::vector<uint32_t> debugMaxEdgeTest;
  std::vector<float> debugDepth, debugIrSum, debugFiltered;

  bool init(int deviceId);
  VulkanBufferResource createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage, bool preferDeviceLocal = true);
  bool createComputeStage(ComputeStage &stage, std::string_view spvResourceName,
                           const std::vector<VulkanBufferResource *> &buffers, uint32_t pushConstantBytes);
  bool createAltDescriptorSet(ComputeStage &stage, const std::vector<VulkanBufferResource *> &buffers);
  bool initTablesAndPipeline();
  void newIrFrame();
  void newDepthFrame();
  bool runFullPipeline(const unsigned char *data9, size_t data9_length);
};

namespace
{
uint32_t findMemoryTypeExact(vk::PhysicalDevice phys, uint32_t typeBits, vk::MemoryPropertyFlags props)
{
  vk::PhysicalDeviceMemoryProperties memProps = phys.getMemoryProperties();
  for (uint32_t i = 0; i < memProps.memoryTypeCount; i++)
  {
    if ((typeBits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & props) == props)
      return i;
  }
  return UINT32_MAX;
}

// Prefer a memory type that also carries `preferred` flags (e.g.
// eDeviceLocal on top of the required host-visible/coherent pair), falling
// back to `required` alone if no such type exists. On discrete GPUs with
// Resizable BAR (confirmed present here via vulkaninfo: the whole
// device-local VRAM heap also has a host-visible+coherent memory type),
// picking the device-local-and-host-visible type instead of the first
// host-visible match keeps every buffer this backend uses fully GPU-local
// instead of routing every GPU-side access over PCIe to system RAM --
// which is what naively taking the first HOST_VISIBLE|HOST_COHERENT match
// does, since that type is enumerated before the ReBAR one.
uint32_t findMemoryType(vk::PhysicalDevice phys, uint32_t typeBits, vk::MemoryPropertyFlags required, vk::MemoryPropertyFlags preferred = {})
{
  if (preferred)
  {
    uint32_t idx = findMemoryTypeExact(phys, typeBits, required | preferred);
    if (idx != UINT32_MAX)
      return idx;
  }
  return findMemoryTypeExact(phys, typeBits, required);
}
} // namespace

VulkanBufferResource VulkanDepthPacketProcessorImpl::createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage, bool preferDeviceLocal)
{
  VulkanBufferResource res;
  res.size = size;

  vk::BufferCreateInfo bufferInfo{};
  bufferInfo.size = size;
  bufferInfo.usage = usage;
  bufferInfo.sharingMode = vk::SharingMode::eExclusive;
  res.buffer = device->createBufferUnique(bufferInfo);

  vk::MemoryRequirements memReq = device->getBufferMemoryRequirements(*res.buffer);
  vk::MemoryPropertyFlags hostProps = vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent;
  vk::MemoryPropertyFlags preferred = preferDeviceLocal ? vk::MemoryPropertyFlagBits::eDeviceLocal : vk::MemoryPropertyFlags{};
  uint32_t memType = findMemoryType(physicalDevice, memReq.memoryTypeBits, hostProps, preferred);

  vk::MemoryAllocateInfo allocInfo{};
  allocInfo.allocationSize = memReq.size;
  allocInfo.memoryTypeIndex = memType;
  res.memory = device->allocateMemoryUnique(allocInfo);

  device->bindBufferMemory(*res.buffer, *res.memory, 0);
  res.mapped = device->mapMemory(*res.memory, 0, size);

  return res;
}

bool VulkanDepthPacketProcessorImpl::init(int deviceId)
{
  try
  {
    vk::ApplicationInfo appInfo{};
    appInfo.pApplicationName = "libfreenect2";
    appInfo.apiVersion = VK_API_VERSION_1_1;

    vk::InstanceCreateInfo instInfo{};
    instInfo.pApplicationInfo = &appInfo;
    instance = vk::createInstanceUnique(instInfo);

    std::vector<vk::PhysicalDevice> devices = instance->enumeratePhysicalDevices();
    if (devices.empty())
    {
      LOG_ERROR("no Vulkan physical devices found");
      return false;
    }

    if (deviceId >= 0 && static_cast<size_t>(deviceId) < devices.size())
      physicalDevice = devices[deviceId];
    else
      physicalDevice = devices[0];

    LOG_INFO("Vulkan device: {}", physicalDevice.getProperties().deviceName.data());

    std::vector<vk::QueueFamilyProperties> queueFamilies = physicalDevice.getQueueFamilyProperties();
    bool foundQueue = false;
    for (uint32_t i = 0; i < queueFamilies.size(); i++)
    {
      if (queueFamilies[i].queueFlags & vk::QueueFlagBits::eCompute)
      {
        queueFamilyIndex = i;
        foundQueue = true;
        break;
      }
    }
    if (!foundQueue)
    {
      LOG_ERROR("no Vulkan compute queue family found");
      return false;
    }

    float queuePriority = 1.0f;
    vk::DeviceQueueCreateInfo queueInfo{};
    queueInfo.queueFamilyIndex = queueFamilyIndex;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &queuePriority;

    vk::DeviceCreateInfo deviceInfo{};
    deviceInfo.queueCreateInfoCount = 1;
    deviceInfo.pQueueCreateInfos = &queueInfo;
    device = physicalDevice.createDeviceUnique(deviceInfo);

    queue = device->getQueue(queueFamilyIndex, 0);

    vk::CommandPoolCreateInfo poolInfo{};
    poolInfo.flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer;
    poolInfo.queueFamilyIndex = queueFamilyIndex;
    commandPool = device->createCommandPoolUnique(poolInfo);

    vk::CommandBufferAllocateInfo cbAllocInfo{};
    cbAllocInfo.commandPool = *commandPool;
    cbAllocInfo.level = vk::CommandBufferLevel::ePrimary;
    cbAllocInfo.commandBufferCount = 1;
    commandBuffer = std::move(device->allocateCommandBuffersUnique(cbAllocInfo).front());

    fence = device->createFenceUnique(vk::FenceCreateInfo{});

    deviceInitialized = true;
    return true;
  }
  catch (const vk::SystemError &e)
  {
    LOG_ERROR("Vulkan initialization failed: {}", e.what());
    return false;
  }
}

bool VulkanDepthPacketProcessorImpl::createComputeStage(ComputeStage &stage, std::string_view spvResourceName,
                                                          const std::vector<VulkanBufferResource *> &buffers,
                                                          uint32_t pushConstantBytes)
{
  size_t length = 0;
  const unsigned char *spv_data = nullptr;
  if (!loadVulkanResource(spvResourceName, &spv_data, &length))
  {
    LOG_ERROR("failed to load embedded Vulkan shader resource: {}", spvResourceName);
    return false;
  }

  vk::ShaderModuleCreateInfo shaderInfo{};
  shaderInfo.codeSize = length;
  shaderInfo.pCode = reinterpret_cast<const uint32_t *>(spv_data);
  stage.shaderModule = device->createShaderModuleUnique(shaderInfo);

  uint32_t n = static_cast<uint32_t>(buffers.size());
  std::vector<vk::DescriptorSetLayoutBinding> bindings(n);
  for (uint32_t i = 0; i < n; i++)
  {
    bindings[i].binding = i;
    bindings[i].descriptorType = vk::DescriptorType::eStorageBuffer;
    bindings[i].descriptorCount = 1;
    bindings[i].stageFlags = vk::ShaderStageFlagBits::eCompute;
  }

  vk::DescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.bindingCount = n;
  layoutInfo.pBindings = bindings.data();
  stage.descSetLayout = device->createDescriptorSetLayoutUnique(layoutInfo);

  vk::PushConstantRange pushRange{};
  pushRange.stageFlags = vk::ShaderStageFlagBits::eCompute;
  pushRange.offset = 0;
  pushRange.size = pushConstantBytes;

  vk::PipelineLayoutCreateInfo pipelineLayoutInfo{};
  pipelineLayoutInfo.setLayoutCount = 1;
  pipelineLayoutInfo.pSetLayouts = &(*stage.descSetLayout);
  if (pushConstantBytes > 0)
  {
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushRange;
  }
  stage.pipelineLayout = device->createPipelineLayoutUnique(pipelineLayoutInfo);

  vk::PipelineShaderStageCreateInfo stageInfo{};
  stageInfo.stage = vk::ShaderStageFlagBits::eCompute;
  stageInfo.module = *stage.shaderModule;
  stageInfo.pName = "main";

  vk::ComputePipelineCreateInfo pipelineInfo{};
  pipelineInfo.stage = stageInfo;
  pipelineInfo.layout = *stage.pipelineLayout;
  auto pipelineResult = device->createComputePipelineUnique(nullptr, pipelineInfo);
  if (pipelineResult.result != vk::Result::eSuccess)
  {
    LOG_ERROR("failed to create Vulkan compute pipeline: {}", spvResourceName);
    return false;
  }
  stage.pipeline = std::move(pipelineResult.value);

  vk::DescriptorPoolSize poolSize{};
  poolSize.type = vk::DescriptorType::eStorageBuffer;
  poolSize.descriptorCount = n;

  vk::DescriptorPoolCreateInfo descPoolInfo{};
  descPoolInfo.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;
  descPoolInfo.maxSets = 1;
  descPoolInfo.poolSizeCount = 1;
  descPoolInfo.pPoolSizes = &poolSize;
  stage.descPool = device->createDescriptorPoolUnique(descPoolInfo);

  vk::DescriptorSetAllocateInfo descAllocInfo{};
  descAllocInfo.descriptorPool = *stage.descPool;
  descAllocInfo.descriptorSetCount = 1;
  descAllocInfo.pSetLayouts = &(*stage.descSetLayout);
  stage.descSet = std::move(device->allocateDescriptorSetsUnique(descAllocInfo).front());

  std::vector<vk::DescriptorBufferInfo> bufferInfos(n);
  std::vector<vk::WriteDescriptorSet> writes(n);
  for (uint32_t i = 0; i < n; i++)
  {
    bufferInfos[i].buffer = *buffers[i]->buffer;
    bufferInfos[i].offset = 0;
    bufferInfos[i].range = buffers[i]->size;

    writes[i].dstSet = *stage.descSet;
    writes[i].dstBinding = i;
    writes[i].descriptorCount = 1;
    writes[i].descriptorType = vk::DescriptorType::eStorageBuffer;
    writes[i].pBufferInfo = &bufferInfos[i];
  }
  device->updateDescriptorSets(writes, {});

  return true;
}

bool VulkanDepthPacketProcessorImpl::createAltDescriptorSet(ComputeStage &stage, const std::vector<VulkanBufferResource *> &buffers)
{
  uint32_t n = static_cast<uint32_t>(buffers.size());

  vk::DescriptorPoolSize poolSize{};
  poolSize.type = vk::DescriptorType::eStorageBuffer;
  poolSize.descriptorCount = n;

  vk::DescriptorPoolCreateInfo descPoolInfo{};
  descPoolInfo.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;
  descPoolInfo.maxSets = 1;
  descPoolInfo.poolSizeCount = 1;
  descPoolInfo.pPoolSizes = &poolSize;
  stage.descPoolAlt = device->createDescriptorPoolUnique(descPoolInfo);

  vk::DescriptorSetAllocateInfo descAllocInfo{};
  descAllocInfo.descriptorPool = *stage.descPoolAlt;
  descAllocInfo.descriptorSetCount = 1;
  descAllocInfo.pSetLayouts = &(*stage.descSetLayout);
  stage.descSetAlt = std::move(device->allocateDescriptorSetsUnique(descAllocInfo).front());

  std::vector<vk::DescriptorBufferInfo> bufferInfos(n);
  std::vector<vk::WriteDescriptorSet> writes(n);
  for (uint32_t i = 0; i < n; i++)
  {
    bufferInfos[i].buffer = *buffers[i]->buffer;
    bufferInfos[i].offset = 0;
    bufferInfos[i].range = buffers[i]->size;

    writes[i].dstSet = *stage.descSetAlt;
    writes[i].dstBinding = i;
    writes[i].descriptorCount = 1;
    writes[i].descriptorType = vk::DescriptorType::eStorageBuffer;
    writes[i].pBufferInfo = &bufferInfos[i];
  }
  device->updateDescriptorSets(writes, {});

  return true;
}

bool VulkanDepthPacketProcessorImpl::initTablesAndPipeline()
{
  try
  {
    lutBuf = createBuffer(2048 * sizeof(int16_t), vk::BufferUsageFlagBits::eStorageBuffer);
    zTableBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer);
    xTableBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer);
    p0TableBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer); // vec4
    dataBuf = createBuffer(RAW_DATA_SIZE, vk::BufferUsageFlagBits::eStorageBuffer);

    aBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    bBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    nBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    // irBuf backs ir_frame->data and gets memcpy'd out to the CPU every
    // single real frame (not just under debugCapture) -- unlike the other
    // intermediate buffers, prefering a device-local memory type here would
    // make that per-frame CPU read hit BAR-mapped VRAM instead of system
    // RAM, which measured dramatically *worse* in practice (see the
    // createBuffer() preferDeviceLocal comment).
    irBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer, /*preferDeviceLocal=*/false);

    aFilteredBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    bFilteredBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    gaussianBuf = createBuffer(9 * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer);
    maxEdgeTestBuf = createBuffer(IMAGE_SIZE * sizeof(uint32_t), vk::BufferUsageFlagBits::eStorageBuffer);

    // depthBuf/filteredBuf: same reasoning as irBuf -- one of the two backs
    // depth_frame->data every real frame (whichever EnableEdgeAwareFilter
    // selects, see runFullPipeline()), so neither should prefer
    // device-local either.
    depthBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer, /*preferDeviceLocal=*/false);
    irSumBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer);
    filteredBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer, /*preferDeviceLocal=*/false);

    std::memcpy(gaussianBuf.mapped, params.gaussian_kernel, 9 * sizeof(float));

    if (!createComputeStage(stage1, "process_pixel_stage1.comp.spv",
                             {&lutBuf, &zTableBuf, &p0TableBuf, &dataBuf, &aBuf, &bBuf, &nBuf, &irBuf},
                             sizeof(float) * (4 + 4 + 2))) // vec4 phase + vec4 ab_mult_per_frq + ab_multiplier + ab_output_multiplier
      return false;

    if (!createComputeStage(filterStage1, "filter_pixel_stage1.comp.spv",
                             {&aBuf, &bBuf, &nBuf, &gaussianBuf, &aFilteredBuf, &bFilteredBuf, &maxEdgeTestBuf},
                             sizeof(float) * 3)) // threshold, exp, max_edge
      return false;

    if (!createComputeStage(stage2, "process_pixel_stage2.comp.spv",
                             {&aFilteredBuf, &bFilteredBuf, &xTableBuf, &zTableBuf, &depthBuf, &irSumBuf},
                             sizeof(float) * 9)) // ab_multiplier, individual_ab_threshold, ab_threshold, ab_confidence_slope/offset, min/max_dealias_confidence, phase_offset, unambigious_dist
      return false;

    // Alternate binding for stage2 reading the raw (unfiltered) a/b buffers
    // directly -- used when EnableBilateralFilter is off, see the
    // ComputeStage::descSetAlt comment above.
    if (!createAltDescriptorSet(stage2, {&aBuf, &bBuf, &xTableBuf, &zTableBuf, &depthBuf, &irSumBuf}))
      return false;

    if (!createComputeStage(filterStage2, "filter_pixel_stage2.comp.spv",
                             {&depthBuf, &irSumBuf, &maxEdgeTestBuf, &filteredBuf},
                             sizeof(float) * 9)) // min/max_depth, 6 edge thresholds, max_edge_count
      return false;

    return true;
  }
  catch (const vk::SystemError &e)
  {
    LOG_ERROR("Vulkan pipeline initialization failed: {}", e.what());
    return false;
  }
}

namespace
{
// vec3-shaped floats (interleaved xyz per pixel, .w padding dropped) --
// used for a/b/n and their filtered counterparts, which all share this
// layout on both the GPU (vec4 storage buffer) and debug/verification side.
void unpackVec3Buffer(const void *mappedVec4, std::vector<float> &out)
{
  out.resize(IMAGE_SIZE * 3);
  const float *src = static_cast<const float *>(mappedVec4);
  for (int i = 0; i < IMAGE_SIZE; i++)
  {
    out[i * 3 + 0] = src[i * 4 + 0];
    out[i * 3 + 1] = src[i * 4 + 1];
    out[i * 3 + 2] = src[i * 4 + 2];
  }
}
} // namespace

void VulkanDepthPacketProcessorImpl::newIrFrame()
{
  ir_frame = std::make_unique<Frame>(512, 424, 4);
  ir_frame->format = Frame::Float;
}

void VulkanDepthPacketProcessorImpl::newDepthFrame()
{
  depth_frame = std::make_unique<Frame>(512, 424, 4);
  depth_frame->format = Frame::Float;
}

bool VulkanDepthPacketProcessorImpl::runFullPipeline(const unsigned char *data9, size_t data9_length)
{
  if (data9_length != RAW_DATA_SIZE)
  {
    LOG_ERROR("unexpected raw depth packet size: {} (expected {})", data9_length, RAW_DATA_SIZE);
    return false;
  }

  std::memcpy(dataBuf.mapped, data9, RAW_DATA_SIZE);

  try
  {
    commandBuffer->reset();

    vk::CommandBufferBeginInfo beginInfo{};
    beginInfo.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
    commandBuffer->begin(beginInfo);

    const uint32_t workGroupSize = 256;
    const uint32_t numGroups = (IMAGE_SIZE + workGroupSize - 1) / workGroupSize;

    // Vulkan (unlike OpenCL's in-order queue) needs an explicit barrier
    // between dependent compute dispatches -- global memory barrier each
    // time since consecutive stages read buffers the previous stage wrote.
    auto barrier = [&]()
    {
      vk::MemoryBarrier memBarrier{};
      memBarrier.srcAccessMask = vk::AccessFlagBits::eShaderWrite;
      memBarrier.dstAccessMask = vk::AccessFlagBits::eShaderRead | vk::AccessFlagBits::eShaderWrite;
      commandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eComputeShader, vk::PipelineStageFlagBits::eComputeShader,
                                      {}, {memBarrier}, {}, {});
    };

    // --- Stage 1: processPixelStage1 ---
    commandBuffer->bindPipeline(vk::PipelineBindPoint::eCompute, *stage1.pipeline);
    commandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eCompute, *stage1.pipelineLayout, 0, {*stage1.descSet}, {});
    {
      struct PushConstants
      {
        float phase[4];
        float abMultiplierPerFrq[4];
        float abMultiplier;
        float abOutputMultiplier;
      } pc{};
      pc.phase[0] = params.phase_in_rad[0];
      pc.phase[1] = params.phase_in_rad[1];
      pc.phase[2] = params.phase_in_rad[2];
      pc.abMultiplierPerFrq[0] = params.ab_multiplier_per_frq[0];
      pc.abMultiplierPerFrq[1] = params.ab_multiplier_per_frq[1];
      pc.abMultiplierPerFrq[2] = params.ab_multiplier_per_frq[2];
      pc.abMultiplier = params.ab_multiplier;
      pc.abOutputMultiplier = params.ab_output_multiplier;
      commandBuffer->pushConstants(*stage1.pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pc), &pc);
    }
    commandBuffer->dispatch(numGroups, 1, 1);
    barrier();

    // --- filterPixelStage1 (bilateral filter) ---
    // If disabled, stage2 below reads directly from the unfiltered a/b
    // buffers via descSetAlt instead. Note max_edge_test is only ever
    // written here -- if this is skipped, filterPixelStage2 later reads
    // whatever max_edge_test held from a previous frame (or uninitialized
    // GPU memory on the very first one). That matches OpenCL's own
    // behavior in this same config combination (buf_edge_test is only
    // written by kernel_filterPixelStage1 there too), not a Vulkan-specific
    // gap.
    if (config.EnableBilateralFilter)
    {
      commandBuffer->bindPipeline(vk::PipelineBindPoint::eCompute, *filterStage1.pipeline);
      commandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eCompute, *filterStage1.pipelineLayout, 0, {*filterStage1.descSet}, {});
      struct PushConstants
      {
        float jointBilateralThreshold;
        float jointBilateralExp;
        float jointBilateralMaxEdge;
      } pc{};
      pc.jointBilateralThreshold = (params.joint_bilateral_ab_threshold * params.joint_bilateral_ab_threshold) /
                                    (params.ab_multiplier * params.ab_multiplier);
      pc.jointBilateralExp = params.joint_bilateral_exp;
      pc.jointBilateralMaxEdge = params.joint_bilateral_max_edge;
      commandBuffer->pushConstants(*filterStage1.pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pc), &pc);
      commandBuffer->dispatch(numGroups, 1, 1);
      barrier();
    }

    // --- processPixelStage2 (phase unwrapping / dealiasing) ---
    commandBuffer->bindPipeline(vk::PipelineBindPoint::eCompute, *stage2.pipeline);
    commandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eCompute, *stage2.pipelineLayout, 0,
                                       {config.EnableBilateralFilter ? *stage2.descSet : *stage2.descSetAlt}, {});
    {
      struct PushConstants
      {
        float abMultiplier;
        float individualAbThreshold;
        float abThreshold;
        float abConfidenceSlope;
        float abConfidenceOffset;
        float minDealiasConfidence;
        float maxDealiasConfidence;
        float phaseOffset;
        float unambigiousDist;
      } pc{};
      pc.abMultiplier = params.ab_multiplier;
      pc.individualAbThreshold = params.individual_ab_threshold;
      pc.abThreshold = params.ab_threshold;
      pc.abConfidenceSlope = params.ab_confidence_slope;
      pc.abConfidenceOffset = params.ab_confidence_offset;
      pc.minDealiasConfidence = params.min_dealias_confidence;
      pc.maxDealiasConfidence = params.max_dealias_confidence;
      pc.phaseOffset = params.phase_offset;
      pc.unambigiousDist = params.unambigious_dist;
      commandBuffer->pushConstants(*stage2.pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pc), &pc);
    }
    commandBuffer->dispatch(numGroups, 1, 1);
    barrier();

    // --- filterPixelStage2 (edge-aware filter) ---
    if (config.EnableEdgeAwareFilter)
    {
      commandBuffer->bindPipeline(vk::PipelineBindPoint::eCompute, *filterStage2.pipeline);
      commandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eCompute, *filterStage2.pipelineLayout, 0, {*filterStage2.descSet}, {});
      struct PushConstants
      {
        float minDepth;
        float maxDepth;
        float edgeAbAvgMinValue;
        float edgeAbStdDevThreshold;
        float edgeCloseDeltaThreshold;
        float edgeFarDeltaThreshold;
        float edgeMaxDeltaThreshold;
        float edgeAvgDeltaThreshold;
        float maxEdgeCount;
      } pc{};
      pc.minDepth = config.MinDepth * 1000.0f;
      pc.maxDepth = config.MaxDepth * 1000.0f;
      pc.edgeAbAvgMinValue = params.edge_ab_avg_min_value;
      pc.edgeAbStdDevThreshold = params.edge_ab_std_dev_threshold;
      pc.edgeCloseDeltaThreshold = params.edge_close_delta_threshold;
      pc.edgeFarDeltaThreshold = params.edge_far_delta_threshold;
      pc.edgeMaxDeltaThreshold = params.edge_max_delta_threshold;
      pc.edgeAvgDeltaThreshold = params.edge_avg_delta_threshold;
      pc.maxEdgeCount = params.max_edge_count;
      commandBuffer->pushConstants(*filterStage2.pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pc), &pc);
      commandBuffer->dispatch(numGroups, 1, 1);
    }

    commandBuffer->end();

    device->resetFences({*fence});

    vk::SubmitInfo submitInfo{};
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &(*commandBuffer);
    queue.submit({submitInfo}, *fence);

    vk::Result waitResult = device->waitForFences({*fence}, VK_TRUE, UINT64_MAX);
    if (waitResult != vk::Result::eSuccess)
    {
      LOG_ERROR("Vulkan fence wait failed");
      return false;
    }

    // Full-image CPU-side unpack/copy for every buffer is only worth
    // paying for during verification (see VulkanStage1Verify) -- doing it
    // unconditionally on every real streamed frame stalled the USB
    // depth-packet-reception thread badly enough to cause real packet loss
    // (the fixed-size buffer pool feeding that thread blocks in
    // allocateBuffer() once this processing thread falls behind).
    if (debugCapture)
    {
      unpackVec3Buffer(aBuf.mapped, debugA);
      unpackVec3Buffer(bBuf.mapped, debugB);
      unpackVec3Buffer(nBuf.mapped, debugN);
      unpackVec3Buffer(aFilteredBuf.mapped, debugFilteredA);
      unpackVec3Buffer(bFilteredBuf.mapped, debugFilteredB);

      debugIr.assign(static_cast<const float *>(irBuf.mapped), static_cast<const float *>(irBuf.mapped) + IMAGE_SIZE);
      debugDepth.assign(static_cast<const float *>(depthBuf.mapped), static_cast<const float *>(depthBuf.mapped) + IMAGE_SIZE);
      debugIrSum.assign(static_cast<const float *>(irSumBuf.mapped), static_cast<const float *>(irSumBuf.mapped) + IMAGE_SIZE);
      debugFiltered.assign(static_cast<const float *>(filteredBuf.mapped), static_cast<const float *>(filteredBuf.mapped) + IMAGE_SIZE);
      debugMaxEdgeTest.assign(static_cast<const uint32_t *>(maxEdgeTestBuf.mapped), static_cast<const uint32_t *>(maxEdgeTestBuf.mapped) + IMAGE_SIZE);
    }

    if (!ir_frame)
      newIrFrame();
    if (!depth_frame)
      newDepthFrame();
    std::memcpy(ir_frame->data, irBuf.mapped, IMAGE_SIZE * sizeof(float));
    const void *finalDepthSrc = config.EnableEdgeAwareFilter ? filteredBuf.mapped : depthBuf.mapped;
    std::memcpy(depth_frame->data, finalDepthSrc, IMAGE_SIZE * sizeof(float));

    return true;
  }
  catch (const vk::SystemError &e)
  {
    LOG_ERROR("Vulkan pipeline dispatch failed: {}", e.what());
    return false;
  }
}

VulkanDepthPacketProcessor::VulkanDepthPacketProcessor(int deviceId) : impl_(std::make_unique<VulkanDepthPacketProcessorImpl>())
{
  if (impl_->init(deviceId))
    impl_->initTablesAndPipeline();
}

VulkanDepthPacketProcessor::~VulkanDepthPacketProcessor() noexcept = default;

void VulkanDepthPacketProcessor::setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config)
{
  DepthPacketProcessor::setConfiguration(config);
  impl_->config = config;
}

void VulkanDepthPacketProcessor::loadP0TablesFromCommandResponse(unsigned char *buffer, size_t buffer_length)
{
  if (!impl_->deviceInitialized)
    return;

  if (buffer_length < sizeof(protocol::P0TablesResponse))
  {
    LOG_ERROR("P0Table response too short!");
    return;
  }

  const auto *p0table = reinterpret_cast<const protocol::P0TablesResponse *>(buffer);
  std::vector<float> p0(static_cast<size_t>(IMAGE_SIZE) * 4);

  for (int r = 0; r < 424; ++r)
  {
    float *it = &p0[static_cast<size_t>(r) * 512 * 4];
    const uint16_t *it0 = &p0table->p0table0[r * 512];
    const uint16_t *it1 = &p0table->p0table1[r * 512];
    const uint16_t *it2 = &p0table->p0table2[r * 512];
    for (int c = 0; c < 512; ++c, it += 4, ++it0, ++it1, ++it2)
    {
      it[0] = -(static_cast<float>(*it0)) * 0.000031f * kPi;
      it[1] = -(static_cast<float>(*it1)) * 0.000031f * kPi;
      it[2] = -(static_cast<float>(*it2)) * 0.000031f * kPi;
      it[3] = 0.0f;
    }
  }

  std::memcpy(impl_->p0TableBuf.mapped, p0.data(), p0.size() * sizeof(float));
}

void VulkanDepthPacketProcessor::loadXZTables(const float *xtable, const float *ztable)
{
  if (!impl_->deviceInitialized)
    return;
  std::memcpy(impl_->xTableBuf.mapped, xtable, static_cast<size_t>(IMAGE_SIZE) * sizeof(float));
  std::memcpy(impl_->zTableBuf.mapped, ztable, static_cast<size_t>(IMAGE_SIZE) * sizeof(float));
  impl_->tablesLoaded = true;
}

void VulkanDepthPacketProcessor::loadLookupTable(const short *lut)
{
  if (!impl_->deviceInitialized)
    return;
  std::memcpy(impl_->lutBuf.mapped, lut, DepthPacketProcessor::LUT_SIZE * sizeof(short));
}

bool VulkanDepthPacketProcessor::good() const
{
  return impl_->deviceInitialized;
}

void VulkanDepthPacketProcessor::process(const DepthPacket &packet)
{
  if (!impl_->deviceInitialized || !impl_->tablesLoaded)
    return;

  if (!impl_->runFullPipeline(packet.buffer, packet.buffer_length))
    return;

  if (!listener_)
    return;

  impl_->ir_frame->timestamp = packet.timestamp;
  impl_->depth_frame->timestamp = packet.timestamp;
  impl_->ir_frame->sequence = packet.sequence;
  impl_->depth_frame->sequence = packet.sequence;

  if (listener_->onNewFrame(Frame::Ir, impl_->ir_frame.get()))
  {
    impl_->ir_frame.release();
    impl_->newIrFrame();
  }
  if (listener_->onNewFrame(Frame::Depth, impl_->depth_frame.get()))
  {
    impl_->depth_frame.release();
    impl_->newDepthFrame();
  }
}

void VulkanDepthPacketProcessor::enableDebugCapture(bool enable)
{
  impl_->debugCapture = enable;
}

const std::vector<float> &VulkanDepthPacketProcessor::debugStage1A() const { return impl_->debugA; }
const std::vector<float> &VulkanDepthPacketProcessor::debugStage1B() const { return impl_->debugB; }
const std::vector<float> &VulkanDepthPacketProcessor::debugStage1N() const { return impl_->debugN; }
const std::vector<float> &VulkanDepthPacketProcessor::debugStage1Ir() const { return impl_->debugIr; }
const std::vector<float> &VulkanDepthPacketProcessor::debugFilteredA() const { return impl_->debugFilteredA; }
const std::vector<float> &VulkanDepthPacketProcessor::debugFilteredB() const { return impl_->debugFilteredB; }
const std::vector<uint32_t> &VulkanDepthPacketProcessor::debugMaxEdgeTest() const { return impl_->debugMaxEdgeTest; }
const std::vector<float> &VulkanDepthPacketProcessor::debugDepth() const { return impl_->debugDepth; }
const std::vector<float> &VulkanDepthPacketProcessor::debugIrSum() const { return impl_->debugIrSum; }
const std::vector<float> &VulkanDepthPacketProcessor::debugFiltered() const { return impl_->debugFiltered; }

} /* namespace libfreenect2 */
