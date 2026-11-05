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

/** @file transfer_pool.cpp Data transfer implementation. */

#include <cstdlib>
#include <cstring>
#include <libfreenect2/usb/transfer_pool.h>
#include <libfreenect2/logging.h>

#include <string>

namespace libfreenect2
{
namespace usb
{

static std::string writeLibusbError(int result)
{
  return std::string(libusb_error_name(result)) + " " + libusb_strerror((libusb_error)result);
}

TransferPool::TransferPool(libusb_device_handle* device_handle, unsigned char device_endpoint) :
    callback_(nullptr),
    device_handle_(device_handle),
    device_endpoint_(device_endpoint),
    buffer_(nullptr),
    buffer_size_(0),
    enable_submit_(false)
{
}

TransferPool::~TransferPool()
{
  deallocate();
}

void TransferPool::enableSubmission()
{
  enable_submit_ = true;
}

void TransferPool::disableSubmission()
{
  enable_submit_ = false;
}

bool TransferPool::enabled() const
{
  return enable_submit_;
}

void TransferPool::deallocate()
{
  for(auto& t : transfers_)
  {
    libusb_free_transfer(t.transfer);
  }
  transfers_.clear();

  if(buffer_ != nullptr)
  {
    buffer_.reset();
    buffer_size_ = 0;
  }
}

bool TransferPool::submit()
{
  if(!enable_submit_)
  {
        LOG_WARNING("transfer submission disabled!");
    return false;
  }

  size_t failcount = 0;
  for(size_t i = 0; i < transfers_.size(); ++i)
  {
    libusb_transfer *transfer = transfers_[i].transfer;
    transfers_[i].setStopped(false);

    int r = libusb_submit_transfer(transfer);

    if(r != LIBUSB_SUCCESS)
    {
            LOG_ERROR("failed to submit transfer: {}", writeLibusbError(r));
      transfers_[i].setStopped(true);
      failcount++;
    }
  }

  if (failcount == transfers_.size())
  {
        LOG_ERROR("all submissions failed. Try debugging with environment variable: LIBUSB_DEBUG=3.");
    return false;
  }

  return true;
}

void TransferPool::cancel()
{
  for(auto& t : transfers_)
  {
    int r = libusb_cancel_transfer(t.transfer);

    if(r != LIBUSB_SUCCESS && r != LIBUSB_ERROR_NOT_FOUND)
    {
            LOG_ERROR("failed to cancel transfer: {}", writeLibusbError(r));
    }
  }

  for(;;)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    size_t stopped_transfers = 0;
    for(auto& t : transfers_)
      stopped_transfers += t.getStopped();
    if (stopped_transfers == transfers_.size())
      break;
        LOG_INFO("waiting for transfer cancellation");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void TransferPool::setCallback(DataCallback *callback)
{
  callback_ = callback;
}

void TransferPool::allocateTransfers(size_t num_transfers, size_t transfer_size)
{
  buffer_size_ = num_transfers * transfer_size;
  buffer_ = std::make_unique<unsigned char[]>(buffer_size_);
  transfers_.reserve(num_transfers);

  unsigned char *ptr = buffer_.get();

  for(size_t i = 0; i < num_transfers; ++i)
  {
    libusb_transfer *transfer = allocateTransfer();
    fillTransfer(transfer);

    transfers_.push_back(TransferPool::Transfer(transfer, this));

    transfer->dev_handle = device_handle_;
    transfer->endpoint = device_endpoint_;
    transfer->buffer = ptr;
    transfer->length = transfer_size;
    transfer->timeout = 1000;
    transfer->callback = (libusb_transfer_cb_fn) &TransferPool::onTransferCompleteStatic;
    transfer->user_data = &transfers_.back();

    ptr += transfer_size;
  }
}

void TransferPool::onTransferCompleteStatic(libusb_transfer* transfer)
{
  TransferPool::Transfer *t = reinterpret_cast<TransferPool::Transfer*>(transfer->user_data);
  t->pool->onTransferComplete(t);
}

void TransferPool::onTransferComplete(TransferPool::Transfer* t)
{
  if(t->transfer->status == LIBUSB_TRANSFER_CANCELLED)
  {
    t->setStopped(true);
    return;
  }

  // process data
  processTransfer(t->transfer);

  if(!enable_submit_)
  {
    t->setStopped(true);
    return;
  }

  // resubmit self
  int r = libusb_submit_transfer(t->transfer);

  if(r != LIBUSB_SUCCESS)
  {
        LOG_ERROR("failed to submit transfer: {}", writeLibusbError(r));
    t->setStopped(true);
  }
}

BulkTransferPool::BulkTransferPool(libusb_device_handle* device_handle, unsigned char device_endpoint) :
    TransferPool(device_handle, device_endpoint)
{
}

BulkTransferPool::~BulkTransferPool()
{
}

void BulkTransferPool::allocate(size_t num_transfers, size_t transfer_size)
{
  allocateTransfers(num_transfers, transfer_size);
}

libusb_transfer* BulkTransferPool::allocateTransfer()
{
  return libusb_alloc_transfer(0);
}

void BulkTransferPool::fillTransfer(libusb_transfer* transfer)
{
  transfer->type = LIBUSB_TRANSFER_TYPE_BULK;
}

void BulkTransferPool::processTransfer(libusb_transfer* transfer)
{
  if(transfer->status != LIBUSB_TRANSFER_COMPLETED) return;

  if(callback_)
    callback_->onDataReceived(transfer->buffer, transfer->actual_length);
}

IsoTransferPool::IsoTransferPool(libusb_device_handle* device_handle, unsigned char device_endpoint) :
    TransferPool(device_handle, device_endpoint),
    num_packets_(0),
    packet_size_(0)
{
}

IsoTransferPool::~IsoTransferPool()
{
}

void IsoTransferPool::allocate(size_t num_transfers, size_t num_packets, size_t packet_size)
{
  num_packets_ = num_packets;
  packet_size_ = packet_size;

  allocateTransfers(num_transfers, num_packets_ * packet_size_);
}

libusb_transfer* IsoTransferPool::allocateTransfer()
{
  return libusb_alloc_transfer(num_packets_);
}

void IsoTransferPool::fillTransfer(libusb_transfer* transfer)
{
  transfer->type = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
  transfer->num_iso_packets = num_packets_;

  libusb_set_iso_packet_lengths(transfer, packet_size_);
}

void IsoTransferPool::processTransfer(libusb_transfer* transfer)
{
  unsigned char *ptr = transfer->buffer;

  // LIBFREENECT2_ISO_DEBUG=1: what do the isochronous transfers really bring? (for transports like USB/IP)
  static const bool iso_debug = std::getenv("LIBFREENECT2_ISO_DEBUG") != nullptr;
  if (iso_debug)
  {
    static unsigned long long transfers = 0;
    if (transfers++ % 400 == 0)
    {
      size_t ok = 0, other = 0, total = 0, nonzero_packets = 0;
      int first_status = 0;
      unsigned char head[8] = {0};
      unsigned char *at = transfer->buffer;
      for (int i = 0; i < transfer->num_iso_packets; ++i)
      {
        auto const &d = transfer->iso_packet_desc[i];
        if (i == 0) first_status = d.status;
        if (d.status == LIBUSB_TRANSFER_COMPLETED) ++ok; else ++other;
        total += d.actual_length;
        if (d.actual_length > 0)
        {
          bool any = false;
          for (unsigned j = 0; j < d.actual_length && !any; ++j) any = at[j] != 0;
          if (any) { if (nonzero_packets++ == 0) std::memcpy(head, at, 8); }
        }
        at += d.length;
      }
      LOG_INFO("iso transfer #{}: {} packets, {} completed, {} other (first status {}), {} bytes in total, {} packets with data other than zeros, first bytes {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
               transfers, transfer->num_iso_packets, ok, other, first_status, total, nonzero_packets, head[0], head[1], head[2], head[3], head[4], head[5], head[6], head[7]);
    }
  }

  for(size_t i = 0; i < num_packets_; ++i)
  {
    if(transfer->iso_packet_desc[i].status != LIBUSB_TRANSFER_COMPLETED) continue;

    if(callback_)
      callback_->onDataReceived(ptr, transfer->iso_packet_desc[i].actual_length);

    ptr += transfer->iso_packet_desc[i].length;
  }
}

} /* namespace usb */
} /* namespace libfreenect2 */

