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

/** @file packet_pipeline_registry.cpp Name-keyed PacketPipeline factory. */

#include <libfreenect2/pipeline_factory.h>
#include <libfreenect2/packet_pipeline.h>

#include <map>
#include <mutex>

namespace libfreenect2
{

namespace
{

struct Entry
{
  PacketPipelineFactory factory;
  bool honors_gpu_device_id = true;
};

class Registry
{
public:
  static Registry &instance()
  {
    static Registry r;
    return r;
  }

  void add(std::string canonical, std::vector<std::string> aliases,
           PacketPipelineFactory factory, bool honors_gpu)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ensureBuiltinsLocked();
    addLocked(std::move(canonical), std::move(aliases), std::move(factory), honors_gpu);
  }

  std::vector<std::string> names()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ensureBuiltinsLocked();
    return order_;
  }

  const Entry *find(std::string_view name)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ensureBuiltinsLocked();

    const std::string key(name);
    auto direct = entries_.find(key);
    if (direct != entries_.end())
      return &direct->second;

    auto alias = alias_to_canonical_.find(key);
    if (alias != alias_to_canonical_.end())
    {
      auto it = entries_.find(alias->second);
      if (it != entries_.end())
        return &it->second;
    }
    return nullptr;
  }

private:
  Registry() = default;

  void addLocked(std::string canonical, std::vector<std::string> aliases,
                 PacketPipelineFactory factory, bool honors_gpu)
  {
    if (!entries_.count(canonical))
      order_.push_back(canonical);
    entries_[canonical] = Entry{std::move(factory), honors_gpu};
    for (auto &a : aliases)
      alias_to_canonical_[std::move(a)] = canonical;
  }

  void ensureBuiltinsLocked()
  {
    if (builtins_done_)
      return;
    builtins_done_ = true;   // set first: registerBuiltinPipeline() re-enters add()

    addLocked("cpu", {},
              [](int) { return std::make_unique<CpuPacketPipeline>(); }, false);

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    addLocked("gl", {"opengl"},
              [](int) { return std::make_unique<OpenGLPacketPipeline>(); }, false);
#endif
#ifdef LIBFREENECT2_WITH_VULKAN_SUPPORT
    addLocked("vk", {"vulkan"},
              [](int id) { return std::make_unique<VulkanPacketPipeline>(id); }, true);
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    addLocked("cl", {"opencl"},
              [](int id) { return std::make_unique<OpenCLPacketPipeline>(id); }, true);
    addLocked("clkde", {"opencl_kde", "cl_kde"},
              [](int id) { return std::make_unique<OpenCLKdePacketPipeline>(id); }, true);
#endif
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    addLocked("cuda", {},
              [](int id) { return std::make_unique<CudaPacketPipeline>(id); }, true);
    addLocked("cudakde", {"cuda_kde"},
              [](int id) { return std::make_unique<CudaKdePacketPipeline>(id); }, true);
#endif
  }

  std::mutex mutex_;
  bool builtins_done_ = false;
  std::map<std::string, Entry> entries_;
  std::map<std::string, std::string> alias_to_canonical_;
  std::vector<std::string> order_;
};

} // namespace

void registerPacketPipeline(std::string canonical_name, std::vector<std::string> aliases,
                            PacketPipelineFactory factory, bool honors_gpu_device_id)
{
  Registry::instance().add(std::move(canonical_name), std::move(aliases),
                           std::move(factory), honors_gpu_device_id);
}

std::vector<std::string> getAvailablePacketPipelines()
{
  return Registry::instance().names();
}

bool hasPacketPipeline(std::string_view name)
{
  return Registry::instance().find(name) != nullptr;
}

bool packetPipelineHonorsGpuDeviceId(std::string_view name)
{
  const Entry *e = Registry::instance().find(name);
  return e != nullptr && e->honors_gpu_device_id;
}

std::unique_ptr<PacketPipeline> createPacketPipeline(std::string_view name, int gpu_device_id)
{
  const Entry *e = Registry::instance().find(name);
  if (e == nullptr)
    return nullptr;
  return e->factory(gpu_device_id);
}

} /* namespace libfreenect2 */
