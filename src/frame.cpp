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

/** @file frame.cpp Frame implementation. Owned by freenect2_logging (not
 *  core freenect2) so every backend can construct Frame subclasses without
 *  linking against freenect2 core. See frame_listener.hpp for why. */

#include <libfreenect2/frame_listener.hpp>

namespace libfreenect2
{

namespace
{
size_t bytesForFormat(Frame::Format format)
{
  switch (format)
  {
    case Frame::Float:
    case Frame::BGRX:
    case Frame::RGBX:
      return 4;
    case Frame::Gray:
      return 1;
    default:   // Raw (size, not per-pixel) / Invalid -- caller should use the other ctor
      return 0;
  }
}
} // namespace

Frame::Frame(size_t width, size_t height, Format format, unsigned char *data_) :
  Frame(width, height, bytesForFormat(format), data_)
{
  this->format = format;
}

Frame::Frame(size_t width, size_t height, size_t bytes_per_pixel, unsigned char *data_) :
  width(width),
  height(height),
  bytes_per_pixel(bytes_per_pixel),
  data(data_),
  exposure(0.f),
  gain(0.f),
  gamma(0.f),
  status(0),
  format(Frame::Invalid),
  rawdata(nullptr)
{
  if (data_)
    return;
  const size_t alignment = 64;
  size_t space = width * height * bytes_per_pixel + alignment;
  rawdata = new unsigned char[space];
  uintptr_t ptr = reinterpret_cast<uintptr_t>(rawdata);
  uintptr_t aligned = (ptr - 1u + alignment) & -alignment;
  data = reinterpret_cast<unsigned char *>(aligned);
}

Frame::~Frame()
{
  delete[] rawdata;
}

} /* namespace libfreenect2 */
