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

/** @file pipeline_factory.h Name-based PacketPipeline construction.
 *
 * One vocabulary for selecting a depth pipeline, shared by the
 * `LIBFREENECT2_PIPELINE` environment variable, the example programs, and the
 * Python bindings. Canonical names are the short tokens the `Protonect` CLI
 * accepts:
 *
 *   cpu  gl  cl  clkde  cuda  cudakde  vk
 *
 * with a few aliases (`opengl`, `opencl`, `opencl_kde`, `cl_kde`, `cuda_kde`,
 * `vulkan`) accepted for convenience.
 *
 * The registry behind createPacketPipeline() is also the seam for a future
 * out-of-tree plugin loader: it would `dlopen`/`LoadLibrary` a backend module
 * and call registerPacketPipeline() from the module's entry point, with no
 * change to this API.
 */

#ifndef LIBFREENECT2_PIPELINE_FACTORY_H_
#define LIBFREENECT2_PIPELINE_FACTORY_H_

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <libfreenect2/config.h>

namespace libfreenect2
{

class PacketPipeline;

///@addtogroup pipeline
///@{

/** Builds a fresh, owned PacketPipeline. `gpu_device_id` selects the GPU for
 * backends that use it (`cl`, `cuda`, `vk`); `-1` means the backend default,
 * and backends that ignore it (`cpu`, `gl`) simply do not read it. */
using PacketPipelineFactory =
    std::function<std::unique_ptr<PacketPipeline>(int gpu_device_id)>;

/** Register a pipeline backend under a canonical name plus optional aliases.
 *
 * Built-in backends register themselves the first time the registry is
 * touched; a plugin loader calls this from a loaded module's entry point.
 * Registering a canonical name that already exists replaces its entry.
 *
 * @param canonical_name        primary lookup name (e.g. `"cuda"`).
 * @param aliases               additional accepted names (e.g. `{"cuda_kde"}`).
 * @param factory               constructs the pipeline.
 * @param honors_gpu_device_id  false for backends that ignore the id (cpu, gl).
 */
LIBFREENECT2_API void registerPacketPipeline(std::string canonical_name,
                                             std::vector<std::string> aliases,
                                             PacketPipelineFactory factory,
                                             bool honors_gpu_device_id = true);

/** Canonical names of every registered pipeline (built-in + plugin), in
 * insertion order of the built-ins followed by any plugin registrations. */
LIBFREENECT2_API std::vector<std::string> getAvailablePacketPipelines();

/** True if `name` (canonical or alias, case-sensitive) resolves to a
 * registered pipeline. */
LIBFREENECT2_API bool hasPacketPipeline(std::string_view name);

/** True if the pipeline registered under `name` reads `gpu_device_id`. */
LIBFREENECT2_API bool packetPipelineHonorsGpuDeviceId(std::string_view name);

/** Construct a pipeline by canonical name or alias.
 * @return the pipeline, or `nullptr` if `name` is not registered.
 */
LIBFREENECT2_API std::unique_ptr<PacketPipeline>
    createPacketPipeline(std::string_view name, int gpu_device_id = -1);

///@}

} /* namespace libfreenect2 */

#endif /* LIBFREENECT2_PIPELINE_FACTORY_H_ */
