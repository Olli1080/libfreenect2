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

/** @file frame_listener_impl.h Implementation of the frame listener classes. */

#ifndef FRAME_LISTENER_IMPL_H_
#define FRAME_LISTENER_IMPL_H_

#include <cstddef>
#include <map>
#include <memory>

#include <libfreenect2/config.h>
#include <libfreenect2/frame_listener.hpp>

namespace libfreenect2
{
///@addtogroup frame
///@{

/** Storage of multiple different types of frames. */
typedef std::map<Frame::Type, Frame*> FrameMap;

/** Look up one frame in a FrameMap without inserting a null entry for a
 * missing type (unlike `map::operator[]`). Returns nullptr if absent. */
inline Frame *getFrame(const FrameMap &frames, Frame::Type type)
{
  auto it = frames.find(type);
  return it != frames.end() ? it->second : nullptr;
}

/** What SyncMultiFrameListener does with a freshly completed frame-set when
 * its buffer is already full (a consumer that can't keep up). */
enum class FrameQueueMode
{
  DropNew,     ///< discard the just-completed set, keep the buffered ones
  DropOldest   ///< evict the oldest buffered set to make room for the new one
};

class SyncMultiFrameListenerImpl;

/** Collect multiple types of frames. */
class LIBFREENECT2_API SyncMultiFrameListener : public FrameListener
{
public:
  /**
   * @param frame_types Use bitwise or to combine multiple types, e.g. `Frame::Ir | Frame::Depth`.
   * @param capacity How many completed frame-sets to buffer for the consumer
   *   (>= 1). With the default 1 a consumer slower than the sensor still gets
   *   a coherent recent set from every waitForNewFrame(), but intermediate
   *   sets are dropped. Raise it to drop fewer frames at the cost of latency
   *   and memory.
   * @param overflow_mode What to drop when the buffer is full (see FrameQueueMode).
   */
  explicit SyncMultiFrameListener(unsigned int frame_types,
                                  std::size_t capacity = 1,
                                  FrameQueueMode overflow_mode = FrameQueueMode::DropNew);
  ~SyncMultiFrameListener() noexcept override;

  /** Test if there are new frames. Non-blocking. */
  bool hasNewFrame() const;

  /** Wait milliseconds for new frames.
   * @param[out] frame Caller is responsible to release the frames in `frame`.
   * @param milliseconds Timeout. This parameter is ignored if not built with C++11 threading support.
   * @return true if a frame is received; false if not.
   */
  bool waitForNewFrame(FrameMap &frame, int milliseconds);

  /** Wait indefinitely for new frames.
   * @param[out] frame Caller is responsible to release the frames in `frame`.
   */
  void waitForNewFrame(FrameMap &frame);

  /** Shortcut to delete all frames in `frame`. */
  void release(FrameMap &frame);

  bool onNewFrame(Frame::Type type, Frame *frame) override;
private:
  std::unique_ptr<SyncMultiFrameListenerImpl> impl_;

  /* Disable copy and assignment constructors */
  SyncMultiFrameListener(const SyncMultiFrameListener&);
  SyncMultiFrameListener& operator=(const SyncMultiFrameListener&);
};

///@}
} /* namespace libfreenect2 */
#endif /* FRAME_LISTENER_IMPL_H_ */
