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

/** @file async_packet_processor.h Asynchronous processing of packets. */

#ifndef ASYNC_PACKET_PROCESSOR_H_
#define ASYNC_PACKET_PROCESSOR_H_

#include <string>

#include <libfreenect2/threading.h>
#include <libfreenect2/packet_processor.h>

namespace libfreenect2
{

/**
 * Packet processor that runs asynchronously.
 * @tparam PacketT Type of the packet being processed.
 */
template<typename PacketT>
class AsyncPacketProcessor : public PacketProcessor<PacketT>
{
public:
  typedef PacketProcessor<PacketT>* PacketProcessorPtr;

  /**
   * Constructor.
   * @param processor Object performing the processing.
   */
  AsyncPacketProcessor(PacketProcessorPtr processor) :
    processor_(processor),
    current_packet_available_(false),
    shutdown_(false),
    thread_(&AsyncPacketProcessor<PacketT>::static_execute, this)
  {
  }

  ~AsyncPacketProcessor() noexcept override
  {
    {
      // shutdown_ is read under packet_mutex_ by the worker; setting it unlocked can lose the wake-up
      std::lock_guard l(packet_mutex_);
      shutdown_ = true;
    }
    packet_condition_.notify_one();

    thread_.join();
  }

  bool ready() const override
  {
    // try to aquire lock, if we succeed the background thread is waiting on packet_condition_
    bool locked = packet_mutex_.try_lock();

    if(locked)
    {
      packet_mutex_.unlock();
    }

    return locked;
  }

  bool good() const override
  {
    return processor_->good();
  }

  void process(const PacketT &packet) override
  {
    {
      std::lock_guard l(packet_mutex_);
      current_packet_ = packet;
      current_packet_available_ = true;
    }
    packet_condition_.notify_one();
  }

  void allocateBuffer(PacketT &p, size_t size) override
  {
    processor_->allocateBuffer(p, size);
  }

  void releaseBuffer(PacketT &p) override
  {
    processor_->releaseBuffer(p);
  }

private:
  PacketProcessorPtr processor_;  ///< The processing routine, executed in the asynchronous thread.
  bool current_packet_available_; ///< Whether #current_packet_ still needs processing.
  PacketT current_packet_;        ///< Packet being processed.

  bool shutdown_;
  mutable std::mutex packet_mutex_; ///< Mutex indicating a new packet can be stored in #current_packet_.
  std::condition_variable packet_condition_; ///< Mutex indicating processing is blocked on lack of packets.
  std::thread thread_; ///< Asynchronous thread.

  /**
   * Wrapper function to start the thread.
   * @param data The #AsyncPacketProcessor object to use.
   */
  static void static_execute(void *data)
  {
    static_cast<AsyncPacketProcessor<PacketT> *>(data)->execute();
  }

  /** Asynchronously process a provided packet. */
  void execute()
  {
    std::string name(processor_->name());
    std::this_thread::set_name(name.c_str());
    std::unique_lock l(packet_mutex_);

    while(!shutdown_)
    {
      packet_condition_.wait(l);

      if(current_packet_available_)
      {
        // invoke process impl
        if (processor_->good())
          processor_->process(current_packet_);
        /*
         * The stream parser passes the buffer asynchronously to processors so
         * it can not wait after process() finishes and free the buffer.  In
         * theory releaseBuffer() should be called as soon as the access to it
         * is finished, but right now no new allocateBuffer() will be called
         * before ready() becomes true, so releaseBuffer() in the main loop of
         * the async processor is OK.
         */
        releaseBuffer(current_packet_);

        current_packet_available_ = false;
      }
    }
  }
};

} /* namespace libfreenect2 */
#endif /* ASYNC_PACKET_PROCESSOR_H_ */
