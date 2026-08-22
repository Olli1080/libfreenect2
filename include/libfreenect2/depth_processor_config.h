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

/** @file depth_processor_config.h Depth processor runtime configuration.
 *  Kept standalone (no dependency on libfreenect2.hpp/Freenect2Device) so
 *  internal depth-processor headers don't have to pull in the full device
 *  API just for this. */

#ifndef LIBFREENECT2_DEPTH_PROCESSOR_CONFIG_H_
#define LIBFREENECT2_DEPTH_PROCESSOR_CONFIG_H_

namespace libfreenect2
{

/** Runtime configuration for depth packet processors. */
struct DepthProcessorConfig
{
  float MinDepth;             ///< Clip at this minimum distance (meter).
  float MaxDepth;             ///< Clip at this maximum distance (meter).

  bool EnableBilateralFilter; ///< Remove some "flying pixels".
  bool EnableEdgeAwareFilter; ///< Remove pixels on edges because ToF cameras produce noisy edges.

  /** Default is 0.5, 4.5, true, true */
  DepthProcessorConfig() :
    MinDepth(0.5f),
    MaxDepth(4.5f), // set to > 8000 for best performance when using the kde pipeline
    EnableBilateralFilter(true),
    EnableEdgeAwareFilter(true)
  {}
};

} /* namespace libfreenect2 */

#endif /* LIBFREENECT2_DEPTH_PROCESSOR_CONFIG_H_ */
