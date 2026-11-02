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

/** @file depth_packet_stream_parser.cpp Parser for getting packets from the depth stream. */

#include <libfreenect2/depth_packet_stream_parser.h>
#include <libfreenect2/logging.h>
#include <memory.h>

namespace libfreenect2
{

DepthPacketStreamParser::DepthPacketStreamParser() :
    frame_stride_(1),
    frames_seen_(0),
    processor_(noopProcessor<DepthPacket>()),
    processed_packets_(-1),
    current_sequence_(0),
    current_subsequence_(0)
{
  size_t single_image = 512*424*11/8;
  buffer_size_ = 10 * single_image;

  processor_->allocateBuffer(packet_, buffer_size_);

  work_buffer_storage_ = std::make_unique<unsigned char[]>(single_image);
  work_buffer_.data = work_buffer_storage_.get();
  work_buffer_.capacity = single_image;
  work_buffer_.length = 0;
}

DepthPacketStreamParser::~DepthPacketStreamParser()
{
}

void DepthPacketStreamParser::setPacketProcessor(libfreenect2::BaseDepthPacketProcessor *processor)
{
  std::lock_guard lock(mutex_);
  processor_->releaseBuffer(packet_);
  if (processor == nullptr)
  {
    // Detached: the shared noop processor has only two buffers and would never get this one back
    processor_ = noopProcessor<DepthPacket>();
    return;
  }
  processor_ = processor;
  processor_->allocateBuffer(packet_, buffer_size_);
}

void DepthPacketStreamParser::setFrameStride(unsigned stride)
{
  frame_stride_ = stride;
}

void DepthPacketStreamParser::onDataReceived(unsigned char* buffer, size_t in_length)
{
  std::lock_guard lock(mutex_);
  if (packet_.memory == nullptr || packet_.memory->data == nullptr)
  {
        LOG_ERROR("Packet buffer is NULL");
    return;
  }
  Buffer &wb = work_buffer_;

  if(in_length == 0)
  {
    //synchronize to subpacket boundary
    wb.length = 0;
  }
  else
  {
    DepthSubPacketFooter *footer = nullptr;
    bool footer_found = false;

    if(wb.length + in_length == wb.capacity + sizeof(DepthSubPacketFooter))
    {
      in_length -= sizeof(DepthSubPacketFooter);
      footer = reinterpret_cast<DepthSubPacketFooter *>(&buffer[in_length]);
      footer_found = true;
    }

    if(wb.length + in_length > wb.capacity)
    {
            LOG_DEBUG("subpacket too large");
      wb.length = 0;
      return;
    }

    memcpy(wb.data + wb.length, buffer, in_length);
    wb.length += in_length;

    if(footer_found)
    {
      if(footer->length != wb.length)
      {
                LOG_DEBUG("image data too short!");
      }
      else
      {
        if(current_sequence_ != footer->sequence)
        {
          if(current_subsequence_ == 0x3ff)
          {
            const unsigned stride = frame_stride_;
            const bool wanted = stride != 0 && frames_seen_++ % stride == 0;

            // Frames dropped on purpose count as handled, or they would be reported as lost
            auto account_packet = [this]()
            {
              processed_packets_++;
              if (processed_packets_ == 0)
                processed_packets_ = current_sequence_;
              int diff = current_sequence_ - processed_packets_;
              const int interval = 30;
              if ((current_sequence_ % interval == 0 && diff != 0) || diff >= interval)
              {
                                LOG_INFO("{} packets were lost", diff);
                processed_packets_ = current_sequence_;
              }
            };

            if(!wanted)
            {
              account_packet();
            }
            else if(processor_->ready())
            {
              DepthPacket &packet = packet_;
              packet.sequence = current_sequence_;
              packet.timestamp = footer->timestamp;
              packet.buffer = packet_.memory->data;
              packet.buffer_length = packet_.memory->capacity;

              processor_->process(packet);
              processor_->allocateBuffer(packet_, buffer_size_);

              account_packet();
            }
            else
            {
                            LOG_DEBUG("skipping depth packet");
            }
          }
          else
          {
                        LOG_DEBUG("not all subsequences received {}", current_subsequence_);
          }

          current_sequence_ = footer->sequence;
          current_subsequence_ = 0;
        }

        Buffer &fb = *packet_.memory;

        // set the bit corresponding to the subsequence number to 1
        current_subsequence_ |= 1 << footer->subsequence;

        if(footer->subsequence * footer->length > fb.capacity)
        {
                    uint32_t const subsequence = footer->subsequence; // a packed field cannot be bound to the logger's reference
                    LOG_DEBUG("front buffer too short! subsequence number is {}", subsequence);
        }
        else
        {
          memcpy(fb.data + (footer->subsequence * footer->length), wb.data + (wb.length - footer->length), footer->length);
        }
      }

      // reset working buffer
      wb.length = 0;
    }
  }
}

} /* namespace libfreenect2 */
