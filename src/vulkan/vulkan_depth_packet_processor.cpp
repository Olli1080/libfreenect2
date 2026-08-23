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

/** @file vulkan_depth_packet_processor.cpp Phase 1 Vulkan depth packet
 * processor -- processPixelStage1 only, see the header's file-level comment.
 *
 * Deliberately uses HOST_VISIBLE|HOST_COHERENT memory with persistent
 * mapping for every buffer (no staging buffers, no device-local memory, no
 * VMA yet) -- simplicity over performance for this phase, since the goal is
 * proving the compute plumbing (buffer upload, descriptor sets, push
 * constants, dispatch, readback) is correct, not making it fast. Revisit
 * once Phase 1 is verified.
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

class VulkanDepthPacketProcessorImpl
{
public:
  bool deviceInitialized = false;
  bool tablesLoaded = false;
  DepthPacketProcessor::Parameters params;

  vk::UniqueInstance instance;
  vk::PhysicalDevice physicalDevice;
  vk::UniqueDevice device;
  vk::Queue queue;
  uint32_t queueFamilyIndex = 0;

  vk::UniqueCommandPool commandPool;
  vk::UniqueCommandBuffer commandBuffer;
  vk::UniqueFence fence;

  vk::UniqueShaderModule stage1Shader;
  vk::UniqueDescriptorSetLayout descSetLayout;
  vk::UniquePipelineLayout pipelineLayout;
  vk::UniquePipeline pipeline;
  vk::UniqueDescriptorPool descPool;
  vk::UniqueDescriptorSet descSet;

  VulkanBufferResource lutBuf, zTableBuf, p0TableBuf, dataBuf;
  VulkanBufferResource aBuf, bBuf, nBuf, irBuf;

  std::vector<float> debugA, debugB, debugN, debugIr;

  bool init(int deviceId);
  VulkanBufferResource createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage);
  bool initTablesAndPipeline();
  bool runStage1(const unsigned char *data9, size_t data9_length);
};

namespace
{
uint32_t findMemoryType(vk::PhysicalDevice phys, uint32_t typeBits, vk::MemoryPropertyFlags props)
{
  vk::PhysicalDeviceMemoryProperties memProps = phys.getMemoryProperties();
  for (uint32_t i = 0; i < memProps.memoryTypeCount; i++)
  {
    if ((typeBits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & props) == props)
      return i;
  }
  return UINT32_MAX;
}
} // namespace

VulkanBufferResource VulkanDepthPacketProcessorImpl::createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage)
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
  uint32_t memType = findMemoryType(physicalDevice, memReq.memoryTypeBits, hostProps);

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

bool VulkanDepthPacketProcessorImpl::initTablesAndPipeline()
{
  try
  {
    lutBuf = createBuffer(2048 * sizeof(int16_t), vk::BufferUsageFlagBits::eStorageBuffer);
    zTableBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer);
    p0TableBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer); // vec4
    dataBuf = createBuffer(RAW_DATA_SIZE, vk::BufferUsageFlagBits::eStorageBuffer);

    aBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    bBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    nBuf = createBuffer(IMAGE_SIZE * sizeof(float) * 4, vk::BufferUsageFlagBits::eStorageBuffer);
    irBuf = createBuffer(IMAGE_SIZE * sizeof(float), vk::BufferUsageFlagBits::eStorageBuffer);

    size_t length = 0;
    const unsigned char *spv_data = nullptr;
    if (!loadVulkanResource("process_pixel_stage1.comp.spv", &spv_data, &length))
    {
      LOG_ERROR("failed to load embedded Vulkan shader resource");
      return false;
    }

    vk::ShaderModuleCreateInfo shaderInfo{};
    shaderInfo.codeSize = length;
    shaderInfo.pCode = reinterpret_cast<const uint32_t *>(spv_data);
    stage1Shader = device->createShaderModuleUnique(shaderInfo);

    std::vector<vk::DescriptorSetLayoutBinding> bindings(8);
    for (uint32_t i = 0; i < bindings.size(); i++)
    {
      bindings[i].binding = i;
      bindings[i].descriptorType = vk::DescriptorType::eStorageBuffer;
      bindings[i].descriptorCount = 1;
      bindings[i].stageFlags = vk::ShaderStageFlagBits::eCompute;
    }

    vk::DescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    descSetLayout = device->createDescriptorSetLayoutUnique(layoutInfo);

    vk::PushConstantRange pushRange{};
    pushRange.stageFlags = vk::ShaderStageFlagBits::eCompute;
    pushRange.offset = 0;
    pushRange.size = sizeof(float) * (4 + 4 + 4); // vec3 phase (padded to vec4) + vec3 ab_mult_per_frq (padded) + ab_multiplier + ab_output_multiplier (+padding)

    vk::PipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &(*descSetLayout);
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushRange;
    pipelineLayout = device->createPipelineLayoutUnique(pipelineLayoutInfo);

    vk::PipelineShaderStageCreateInfo stageInfo{};
    stageInfo.stage = vk::ShaderStageFlagBits::eCompute;
    stageInfo.module = *stage1Shader;
    stageInfo.pName = "main";

    vk::ComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = *pipelineLayout;
    auto pipelineResult = device->createComputePipelineUnique(nullptr, pipelineInfo);
    if (pipelineResult.result != vk::Result::eSuccess)
    {
      LOG_ERROR("failed to create Vulkan compute pipeline");
      return false;
    }
    pipeline = std::move(pipelineResult.value);

    vk::DescriptorPoolSize poolSize{};
    poolSize.type = vk::DescriptorType::eStorageBuffer;
    poolSize.descriptorCount = 8;

    vk::DescriptorPoolCreateInfo descPoolInfo{};
    descPoolInfo.flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet;
    descPoolInfo.maxSets = 1;
    descPoolInfo.poolSizeCount = 1;
    descPoolInfo.pPoolSizes = &poolSize;
    descPool = device->createDescriptorPoolUnique(descPoolInfo);

    vk::DescriptorSetAllocateInfo descAllocInfo{};
    descAllocInfo.descriptorPool = *descPool;
    descAllocInfo.descriptorSetCount = 1;
    descAllocInfo.pSetLayouts = &(*descSetLayout);
    descSet = std::move(device->allocateDescriptorSetsUnique(descAllocInfo).front());

    VulkanBufferResource *buffers[8] = {&lutBuf, &zTableBuf, &p0TableBuf, &dataBuf, &aBuf, &bBuf, &nBuf, &irBuf};
    std::vector<vk::DescriptorBufferInfo> bufferInfos(8);
    std::vector<vk::WriteDescriptorSet> writes(8);
    for (uint32_t i = 0; i < 8; i++)
    {
      bufferInfos[i].buffer = *buffers[i]->buffer;
      bufferInfos[i].offset = 0;
      bufferInfos[i].range = buffers[i]->size;

      writes[i].dstSet = *descSet;
      writes[i].dstBinding = i;
      writes[i].descriptorCount = 1;
      writes[i].descriptorType = vk::DescriptorType::eStorageBuffer;
      writes[i].pBufferInfo = &bufferInfos[i];
    }
    device->updateDescriptorSets(writes, {});

    return true;
  }
  catch (const vk::SystemError &e)
  {
    LOG_ERROR("Vulkan pipeline initialization failed: {}", e.what());
    return false;
  }
}

bool VulkanDepthPacketProcessorImpl::runStage1(const unsigned char *data9, size_t data9_length)
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

    commandBuffer->bindPipeline(vk::PipelineBindPoint::eCompute, *pipeline);
    commandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eCompute, *pipelineLayout, 0, {*descSet}, {});

    struct PushConstants
    {
      float phase[4];
      float abMultiplierPerFrq[4];
      float abMultiplier;
      float abOutputMultiplier;
      float pad[2];
    } pc{};
    pc.phase[0] = params.phase_in_rad[0];
    pc.phase[1] = params.phase_in_rad[1];
    pc.phase[2] = params.phase_in_rad[2];
    pc.abMultiplierPerFrq[0] = params.ab_multiplier_per_frq[0];
    pc.abMultiplierPerFrq[1] = params.ab_multiplier_per_frq[1];
    pc.abMultiplierPerFrq[2] = params.ab_multiplier_per_frq[2];
    pc.abMultiplier = params.ab_multiplier;
    pc.abOutputMultiplier = params.ab_output_multiplier;

    commandBuffer->pushConstants(*pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pc), &pc);

    const uint32_t workGroupSize = 256;
    uint32_t numGroups = (IMAGE_SIZE + workGroupSize - 1) / workGroupSize;
    commandBuffer->dispatch(numGroups, 1, 1);

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

    debugA.resize(IMAGE_SIZE * 3);
    debugB.resize(IMAGE_SIZE * 3);
    debugN.resize(IMAGE_SIZE * 3);
    debugIr.resize(IMAGE_SIZE);

    const float *aSrc = static_cast<const float *>(aBuf.mapped);
    const float *bSrc = static_cast<const float *>(bBuf.mapped);
    const float *nSrc = static_cast<const float *>(nBuf.mapped);
    const float *irSrc = static_cast<const float *>(irBuf.mapped);
    for (int i = 0; i < IMAGE_SIZE; i++)
    {
      debugA[i * 3 + 0] = aSrc[i * 4 + 0];
      debugA[i * 3 + 1] = aSrc[i * 4 + 1];
      debugA[i * 3 + 2] = aSrc[i * 4 + 2];
      debugB[i * 3 + 0] = bSrc[i * 4 + 0];
      debugB[i * 3 + 1] = bSrc[i * 4 + 1];
      debugB[i * 3 + 2] = bSrc[i * 4 + 2];
      debugN[i * 3 + 0] = nSrc[i * 4 + 0];
      debugN[i * 3 + 1] = nSrc[i * 4 + 1];
      debugN[i * 3 + 2] = nSrc[i * 4 + 2];
      debugIr[i] = irSrc[i];
    }

    return true;
  }
  catch (const vk::SystemError &e)
  {
    LOG_ERROR("Vulkan stage1 dispatch failed: {}", e.what());
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
  // xtable is loaded but unused until stage 2 is ported; z_table is the
  // only one processPixelStage1 actually reads.
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
  // Phase 1: run stage 1 only and populate the debug buffers for
  // verification against the OpenCL backend's own stage-1 output. Does not
  // yet produce a depth/ir Frame or notify listener_ -- see the header's
  // file-level comment.
  if (!impl_->deviceInitialized || !impl_->tablesLoaded)
    return;

  impl_->runStage1(packet.buffer, packet.buffer_length);
}

Allocator *VulkanDepthPacketProcessor::getAllocator()
{
  return nullptr;
}

const std::vector<float> &VulkanDepthPacketProcessor::debugStage1A() const { return impl_->debugA; }
const std::vector<float> &VulkanDepthPacketProcessor::debugStage1B() const { return impl_->debugB; }
const std::vector<float> &VulkanDepthPacketProcessor::debugStage1N() const { return impl_->debugN; }
const std::vector<float> &VulkanDepthPacketProcessor::debugStage1Ir() const { return impl_->debugIr; }

} /* namespace libfreenect2 */
