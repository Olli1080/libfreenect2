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

/** @file frame_listener_impl.cpp Implementation classes for frame listeners. */

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <cstring>
#include <deque>
#include <utility>

namespace libfreenect2
{

FrameListener::~FrameListener() {}

namespace
{
std::size_t frameBytes(const Frame *f)
{
  // Raw repurposes bytes_per_pixel as the total buffer size (see frame_listener.hpp).
  return f->format == Frame::Raw ? f->bytes_per_pixel
                                 : f->width * f->height * f->bytes_per_pixel;
}

Frame *copyFrame(const Frame *src)
{
  Frame *dst = new Frame(src->width, src->height, src->bytes_per_pixel);
  std::memcpy(dst->data, src->data, frameBytes(src));
  dst->timestamp = src->timestamp;
  dst->sequence = src->sequence;
  dst->exposure = src->exposure;
  dst->gain = src->gain;
  dst->gamma = src->gamma;
  dst->status = src->status;
  dst->format = src->format;
  return dst;
}
} // namespace

/** Implementation class for synchronizing different types of frames.
 *
 * Two modes, chosen by `capacity`:
 *
 *  - capacity == 1 (default): zero-copy. `onNewFrame` returns false (the
 *    pipeline keeps and reuses its buffer) whenever the one buffered set has
 *    not yet been release()d. Exactly the historical behaviour -- a consumer
 *    must release before the next waitForNewFrame() delivers, and holding a
 *    set never stalls the pipeline beyond its own 2-deep buffer pool.
 *
 *  - capacity >= 2: the listener copies each frame into its own storage and
 *    returns false (pipeline reuses its buffer immediately), then queues up
 *    to `capacity` completed sets. A slow consumer drops intermediate sets
 *    per `overflow_` but never blocks the pipeline, and may hold sets for as
 *    long as it likes.
 */
class SyncMultiFrameListenerImpl
{
public:
  std::mutex mutex_;
  std::condition_variable condition_;

  const unsigned int subscribed_frame_types_;
  const std::size_t capacity_;
  const FrameQueueMode overflow_;
  const bool copy_;

  FrameMap building_;
  unsigned int building_frame_types_ = 0;
  std::deque<FrameMap> ready_;
  bool checked_out_ = false;   // capacity_ == 1 only: a set is out, awaiting release()

  SyncMultiFrameListenerImpl(unsigned int frame_types, std::size_t capacity, FrameQueueMode overflow) :
    subscribed_frame_types_(frame_types),
    capacity_(capacity < 1 ? 1 : capacity),
    overflow_(overflow),
    copy_(capacity_ >= 2)
  {
  }

  bool hasNewFrame() const
  {
    return !ready_.empty();
  }

  static void releaseMap(FrameMap &m)
  {
    for (auto &entry : m)
    {
      delete entry.second;
      entry.second = nullptr;
    }
    m.clear();
  }

  void drainAll()
  {
    releaseMap(building_);
    building_frame_types_ = 0;
    for (auto &m : ready_)
      releaseMap(m);
    ready_.clear();
  }
};

SyncMultiFrameListener::SyncMultiFrameListener(unsigned int frame_types, std::size_t capacity,
                                               FrameQueueMode overflow_mode) :
    impl_(std::make_unique<SyncMultiFrameListenerImpl>(frame_types, capacity, overflow_mode))
{
}

SyncMultiFrameListener::~SyncMultiFrameListener()
{
  std::scoped_lock l(impl_->mutex_);
  impl_->drainAll();
}

bool SyncMultiFrameListener::hasNewFrame() const
{
  std::unique_lock l(impl_->mutex_);

  return impl_->hasNewFrame();
}

bool SyncMultiFrameListener::waitForNewFrame(FrameMap &frame, int milliseconds)
{
  std::unique_lock l(impl_->mutex_);

  if (impl_->condition_.wait_for(l, std::chrono::milliseconds(milliseconds),
                                 [this] { return impl_->hasNewFrame(); }))
  {
    frame = std::move(impl_->ready_.front());
    impl_->ready_.pop_front();
    if (!impl_->copy_)
      impl_->checked_out_ = true;
    return true;
  }
  return false;
}

void SyncMultiFrameListener::waitForNewFrame(FrameMap &frame)
{
  std::unique_lock l(impl_->mutex_);

  impl_->condition_.wait(l, [this] { return impl_->hasNewFrame(); });

  frame = std::move(impl_->ready_.front());
  impl_->ready_.pop_front();
  if (!impl_->copy_)
    impl_->checked_out_ = true;
}

void SyncMultiFrameListener::release(FrameMap &frame)
{
  for (auto &entry : frame)
  {
    delete entry.second;
    entry.second = nullptr;
  }
  frame.clear();

  if (!impl_->copy_)
  {
    std::scoped_lock l(impl_->mutex_);
    impl_->checked_out_ = false;
  }
}

bool SyncMultiFrameListener::onNewFrame(Frame::Type type, Frame *frame)
{
  if ((impl_->subscribed_frame_types_ & type) == 0)
    return false;

  {
    std::scoped_lock l(impl_->mutex_);

    // Zero-copy mode: refuse (pipeline reuses its buffer) while the consumer
    // still holds the previous set, or while a completed set is waiting.
    if (!impl_->copy_ && (impl_->checked_out_ || !impl_->ready_.empty()))
      return false;

    Frame *stored = impl_->copy_ ? copyFrame(frame) : frame;

    auto it = impl_->building_.find(type);
    if (it != impl_->building_.end())
    {
      delete it->second;   // replace an earlier frame of the same type
      it->second = stored;
    }
    else
    {
      impl_->building_[type] = stored;
    }
    impl_->building_frame_types_ |= type;

    if (impl_->building_frame_types_ == impl_->subscribed_frame_types_)
    {
      if (impl_->ready_.size() >= impl_->capacity_)
      {
        if (impl_->overflow_ == FrameQueueMode::DropNew)
        {
          SyncMultiFrameListenerImpl::releaseMap(impl_->building_);
          impl_->building_frame_types_ = 0;
          return impl_->copy_;   // copy mode: we own the copies (freed); zero-copy: pipeline keeps its buffer
        }
        SyncMultiFrameListenerImpl::releaseMap(impl_->ready_.front());
        impl_->ready_.pop_front();
      }

      impl_->ready_.push_back(std::move(impl_->building_));
      impl_->building_.clear();
      impl_->building_frame_types_ = 0;
    }
  }

  impl_->condition_.notify_one();

  return !impl_->copy_ ? true : false;   // zero-copy: took ownership of the buffer; copy: pipeline keeps it
}

} /* namespace libfreenect2 */
