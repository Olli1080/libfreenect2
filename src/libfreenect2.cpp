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

/** @file libfreenect2.cpp Freenect2 devices and processing implementation. */

#include <string>
#include <string_view>
#include <vector>
#include <algorithm>
#include <libusb.h>
#include <limits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>

#include <libfreenect2/libfreenect2.hpp>

#include <libfreenect2/usb/event_loop.h>
#include <libfreenect2/usb/transfer_pool.h>
#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/rgb_packet_processor.h>
#include <libfreenect2/protocol/usb_control.h>
#include <libfreenect2/protocol/command.h>
#include <libfreenect2/protocol/response.h>
#include <libfreenect2/protocol/command_transaction.h>
#include <libfreenect2/logging.h>
#include <libfreenect2/threading.h>

namespace libfreenect2
{
using namespace libfreenect2;
using namespace libfreenect2::usb;
using namespace libfreenect2::protocol;

static std::string writeLibusbError(int result)
{
  return std::string(libusb_error_name(result)) + " " + libusb_strerror((libusb_error)result);
}

/*
For detailed analysis see https://github.com/OpenKinect/libfreenect2/issues/144

The following discussion is in no way authoritative. It is the current best
explanation considering the hardcoded parameters and decompiled code.

p0 tables are the "initial shift" of phase values, as in US8587771 B2.

Three p0 tables are used for "disamgibuation" in the first half of stage 2
processing.

At the end of stage 2 processing:

phase_final is the phase shift used to compute the travel distance.

What is being measured is max_depth (d), the total travel distance of the
reflected ray.

But what we want is depth_fit (z), the distance from reflection to the XY
plane. There are two issues: the distance before reflection is not needed;
and the measured ray is not normal to the XY plane.

Suppose L is the distance between the light source and the focal point (a
fixed constant), and xu,yu is the undistorted and normalized coordinates for
each measured pixel at unit depth.

Through some derivation, we have

    z = (d*d - L*L)/(d*sqrt(xu*xu + yu*yu + 1) - xu*L)/2.

The expression in stage 2 processing is a variant of this, with the term
`-L*L` removed. Detailed derivation can be found in the above issue.

Here, the two terms `sqrt(xu*xu + yu*yu + 1)` and `xu` requires undistorted
coordinates, which is hard to compute in real-time because the inverse of
radial and tangential distortion has no analytical solutions and requires
numeric methods to solve. Thus these two terms are precomputed once and
their variants are stored as ztable and xtable respectively.

Even though x/ztable is derived with undistortion, they are only used to
correct the effect of distortion on the z value. Image warping is needed for
correcting distortion on x-y value, which happens in registration.cpp.
*/
struct IrCameraTables: Freenect2Device::IrCameraParams
{
  std::vector<float> xtable;
  std::vector<float> ztable;
  std::vector<short> lut;

  IrCameraTables(const Freenect2Device::IrCameraParams &parent):
    Freenect2Device::IrCameraParams(parent),
    xtable(DepthPacketProcessor::TABLE_SIZE),
    ztable(DepthPacketProcessor::TABLE_SIZE),
    lut(DepthPacketProcessor::LUT_SIZE)
  {
    const double scaling_factor = 8192;
    const double unambigious_dist = 6250.0/3;
    size_t divergence = 0;
    for (size_t i = 0; i < DepthPacketProcessor::TABLE_SIZE; i++)
    {
      size_t xi = i % 512;
      size_t yi = i / 512;
      double xd = (xi + 0.5 - cx)/fx;
      double yd = (yi + 0.5 - cy)/fy;
      double xu, yu;
      divergence += !undistort(xd, yd, xu, yu);
      xtable[i] = scaling_factor*xu;
      ztable[i] = unambigious_dist/sqrt(xu*xu + yu*yu + 1);
    }

    if (divergence > 0)
            LOG_ERROR("{} pixels in x/ztable have incorrect undistortion.", divergence);

    short y = 0;
    for (int x = 0; x < 1024; x++)
    {
      unsigned inc = 1 << (x/128 - (x>=128));
      lut[x] = y;
      lut[1024 + x] = -y;
      y += inc;
    }
    lut[1024] = 32767;
  }

  //x,y: undistorted, normalized coordinates
  //xd,yd: distorted, normalized coordinates
  void distort(double x, double y, double &xd, double &yd) const
  {
    double x2 = x * x;
    double y2 = y * y;
    double r2 = x2 + y2;
    double xy = x * y;
    double kr = ((k3 * r2 + k2) * r2 + k1) * r2 + 1.0;
    xd = x*kr + p2*(r2 + 2*x2) + 2*p1*xy;
    yd = y*kr + p1*(r2 + 2*y2) + 2*p2*xy;
  }

  //The inverse of distort() using Newton's method
  //Return true if converged correctly
  //This function considers tangential distortion with double precision.
  bool undistort(double x, double y, double &xu, double &yu) const
  {
    double x0 = x;
    double y0 = y;

    double last_x = x;
    double last_y = y;
    const int max_iterations = 100;
    int iter;
    for (iter = 0; iter < max_iterations; iter++) {
      double x2 = x*x;
      double y2 = y*y;
      double x2y2 = x2 + y2;
      double x2y22 = x2y2*x2y2;
      double x2y23 = x2y2*x2y22;

      //Jacobian matrix
      double Ja = k3*x2y23 + (k2+6*k3*x2)*x2y22 + (k1+4*k2*x2)*x2y2 + 2*k1*x2 + 6*p2*x + 2*p1*y + 1;
      double Jb = 6*k3*x*y*x2y22 + 4*k2*x*y*x2y2 + 2*k1*x*y + 2*p1*x + 2*p2*y;
      double Jc = Jb;
      double Jd = k3*x2y23 + (k2+6*k3*y2)*x2y22 + (k1+4*k2*y2)*x2y2 + 2*k1*y2 + 2*p2*x + 6*p1*y + 1;

      //Inverse Jacobian
      double Jdet = 1/(Ja*Jd - Jb*Jc);
      double a = Jd*Jdet;
      double b = -Jb*Jdet;
      double c = -Jc*Jdet;
      double d = Ja*Jdet;

      double f, g;
      distort(x, y, f, g);
      f -= x0;
      g -= y0;

      x -= a*f + b*g;
      y -= c*f + d*g;
      const double eps = std::numeric_limits<double>::epsilon()*16;
      if (fabs(x - last_x) <= eps && fabs(y - last_y) <= eps)
        break;
      last_x = x;
      last_y = y;
    }
    xu = x;
    yu = y;
    return iter < max_iterations;
  }
};

struct LibusbContextDeleter
{
  bool managed;

  void operator()(libusb_context *ctx) const
  {
    if (managed && ctx != nullptr)
      libusb_exit(ctx);
  }
};

struct LibusbHandleDeleter
{
  void operator()(libusb_device_handle *handle) const
  {
    if (handle != nullptr)
      libusb_close(handle);
  }
};

struct LibusbDeviceDeleter
{
  void operator()(libusb_device *device) const
  {
    if (device != nullptr)
      libusb_unref_device(device);
  }
};

/** Freenect2 device implementation. */
class Freenect2DeviceImpl : public Freenect2Device
{
private:
  enum State
  {
    Created,
    Open,
    Streaming,
    Closed
  };

  State state_;
  bool has_usb_interfaces_;

  Freenect2Impl *context_;
  std::unique_ptr<libusb_device, LibusbDeviceDeleter> usb_device_;
  std::unique_ptr<libusb_device_handle, LibusbHandleDeleter> usb_device_handle_;

  BulkTransferPool rgb_transfer_pool_;
  IsoTransferPool ir_transfer_pool_;

  UsbControl usb_control_;
  CommandTransaction command_tx_;
  int command_seq_;

  std::unique_ptr<const PacketPipeline> pipeline_;
  std::string serial_, firmware_;
  Freenect2Device::IrCameraParams ir_camera_params_;
  Freenect2Device::ColorCameraParams rgb_camera_params_;
public:
  Freenect2DeviceImpl(Freenect2Impl *context, std::unique_ptr<const PacketPipeline> pipeline, libusb_device *usb_device, libusb_device_handle *usb_device_handle, const std::string &serial);
  ~Freenect2DeviceImpl() override;

  bool isSameUsbDevice(libusb_device* other);

  std::string getSerialNumber() override;
  std::string getFirmwareVersion() override;

  Freenect2Device::ColorCameraParams getColorCameraParams() override;
  Freenect2Device::IrCameraParams getIrCameraParams() override;
  void setColorCameraParams(const Freenect2Device::ColorCameraParams &params) override;
  void setIrCameraParams(const Freenect2Device::IrCameraParams &params) override;
  void setConfiguration(const Freenect2Device::Config &config) override;

  int nextCommandSeq();

  bool open();

  void setColorFrameListener(libfreenect2::FrameListener* rgb_frame_listener) override;
  void setIrAndDepthFrameListener(libfreenect2::FrameListener* ir_frame_listener) override;
  void setColorAutoExposure(float exposure_compensation = 0) override;
  void setColorSemiAutoExposure(float pseudo_exposure_time_ms) override;
  void setColorManualExposure(float integration_time_ms, float analog_gain) override;
  void setColorSetting(ColorSettingCommandType cmd, uint32_t value) override;
  void setColorSetting(ColorSettingCommandType cmd, float value) override;
  uint32_t getColorSetting(ColorSettingCommandType cmd) override;
  float getColorSettingFloat(ColorSettingCommandType cmd) override;
  void setLedStatus(LedSettings led) override;
  bool start() override;
  bool startStreams(bool rgb, bool depth) override;
  bool stop() override;
  bool close() override;
};

class Freenect2ReplayDevice : public Freenect2Device
{
public:
  Freenect2ReplayDevice(Freenect2ReplayImpl *context_, const std::vector<std::string>& frame_filenames, std::unique_ptr<const PacketPipeline> pipeline);
  ~Freenect2ReplayDevice() override;

  std::string getSerialNumber() override;
  std::string getFirmwareVersion() override;

  ColorCameraParams getColorCameraParams() override;
  IrCameraParams getIrCameraParams() override;
  void setColorCameraParams(const ColorCameraParams &params) override;
  void setIrCameraParams(const IrCameraParams &params) override;
  void setConfiguration(const Config &config) override;

  void setColorFrameListener(FrameListener* listener) override;
  void setIrAndDepthFrameListener(FrameListener* listener) override;

  void setColorAutoExposure(float exposure_compensation) override {}
  void setColorSemiAutoExposure(float pseudo_exposure_time_ms) override {}
  void setColorManualExposure(float integration_time_ms, float analog_gain) override {}
  void setColorSetting(ColorSettingCommandType cmd, uint32_t value) override {}
  void setColorSetting(ColorSettingCommandType cmd, float value) override {}
  uint32_t getColorSetting(ColorSettingCommandType cmd) override { return 0u; }
  float getColorSettingFloat(ColorSettingCommandType cmd) override { return 0.0f; }
  void setLedStatus(LedSettings led) override {}

  bool open();

  bool start() override;
  bool startStreams(bool rgb, bool depth) override;
  bool stop() override;
  bool close() override;

  // X, Z, LUT tables are generated in setIrCameraParams().
  void loadP0Tables(unsigned char* buffer, size_t buffer_length);

private:
  bool processRawFrame(Frame::Type type, Frame* frame);
  void processRgbFrame(Frame* frame);
  void processDepthFrame(Frame* frame);

  void run();
  static void static_execute(void* arg);

  Freenect2ReplayImpl *context_;
  std::unique_ptr<const PacketPipeline> pipeline_;
  size_t buffer_size_;
  DepthPacket packet_;

  std::vector<std::string> frame_filenames_;
  std::unique_ptr<std::thread> t_;
  bool running_;

  Freenect2Device::IrCameraParams ir_camera_params_;
  Freenect2Device::ColorCameraParams rgb_camera_params_;
};

static std::string printBusAndDevice(libusb_device *dev, int status = 0)
{
  std::string out = "@" + std::to_string(int(libusb_get_bus_number(dev))) + ":" + std::to_string(int(libusb_get_device_address(dev)));
  if (status)
    out += " " + writeLibusbError(status);
  return out;
}

/** Freenect2 device storage and control. */
class Freenect2Impl
{
private:
  bool managed_usb_context_;
  std::unique_ptr<libusb_context, LibusbContextDeleter> usb_context_;
  EventLoop usb_event_loop_;
public:
  struct UsbDeviceWithSerial
  {
    std::unique_ptr<libusb_device, LibusbDeviceDeleter> dev;
    std::string serial;
  };
  typedef std::vector<UsbDeviceWithSerial> UsbDeviceVector;
  typedef std::vector<Freenect2DeviceImpl *> DeviceVector;

  bool has_device_enumeration_;
  UsbDeviceVector enumerated_devices_;
  DeviceVector devices_;

  bool initialized;

  Freenect2Impl(void *usb_context) :
    managed_usb_context_(usb_context == nullptr),
    usb_context_(reinterpret_cast<libusb_context *>(usb_context), LibusbContextDeleter{usb_context == nullptr}),
    has_device_enumeration_(false),
    initialized(false)
  {
#ifdef __linux__
    if (libusb_get_version()->nano < 10952)
    {
            LOG_ERROR("Your libusb does not support large iso buffer!");
      return;
    }
#endif

    if(managed_usb_context_)
    {
      libusb_context *ctx = nullptr;
      int r = libusb_init(&ctx);
      if(r != 0)
      {
                LOG_ERROR("failed to create usb context: {}", writeLibusbError(r));
        return;
      }

      usb_context_ = std::unique_ptr<libusb_context, LibusbContextDeleter>(ctx, LibusbContextDeleter{true});

#if defined(_WIN32) || defined (__WIN32__) || defined(__WINDOWS__)
      (void)libusb_set_option(usb_context_.get(), LIBUSB_OPTION_USE_USBDK);
#endif
    }

    usb_event_loop_.start(usb_context_.get());
    initialized = true;
  }

  ~Freenect2Impl() noexcept
  {
    if (!initialized)
      return;

    clearDevices();
    clearDeviceEnumeration();

    usb_event_loop_.stop();
  }

  void addDevice(Freenect2DeviceImpl *device)
  {
    if (!initialized)
      return;

    devices_.push_back(device);
  }

  void removeDevice(Freenect2DeviceImpl *device)
  {
    if (!initialized)
      return;

    auto it = std::find(devices_.begin(), devices_.end(), device);

    if(it != devices_.end())
    {
      devices_.erase(it);
    }
    else
    {
            LOG_WARNING("tried to remove device, which is not in the internal device list!");
    }
  }

  bool tryGetDevice(libusb_device *usb_device, Freenect2DeviceImpl **device)
  {
    if (!initialized)
      return false;

    for(auto dev : devices_)
    {
      if(dev->isSameUsbDevice(usb_device))
      {
        *device = dev;
        return true;
      }
    }

    return false;
  }

  void clearDevices()
  {
    if (!initialized)
      return;

    DeviceVector devices(devices_.begin(), devices_.end());

    for(auto dev : devices)
    {
      delete dev;
    }

    if(!devices_.empty())
    {
            LOG_WARNING("after deleting all devices the internal device list should be empty!");
    }
  }

  void clearDeviceEnumeration()
  {
    if (!initialized)
      return;

    // enumerated device refs are released by the unique_ptr members, this should not affect opened devices
    enumerated_devices_.clear();
    has_device_enumeration_ = false;
  }

  void enumerateDevices()
  {
    if (!initialized)
      return;

        LOG_INFO("enumerating devices...");
    libusb_device **device_list;
    int num_devices = libusb_get_device_list(usb_context_.get(), &device_list);

        LOG_INFO("{} usb devices connected", num_devices);

    if(num_devices > 0)
    {
      for(int idx = 0; idx < num_devices; ++idx)
      {
        libusb_device *dev = device_list[idx];
        libusb_device_descriptor dev_desc;

        int r = libusb_get_device_descriptor(dev, &dev_desc); // this is always successful

        if(dev_desc.idVendor == Freenect2Device::VendorId && (dev_desc.idProduct == Freenect2Device::ProductId || dev_desc.idProduct == Freenect2Device::ProductIdPreview))
        {
          Freenect2DeviceImpl *freenect2_dev;

          // prevent error if device is already open
          if(tryGetDevice(dev, &freenect2_dev))
          {
            UsbDeviceWithSerial dev_with_serial;
            dev_with_serial.dev.reset(dev);
            dev_with_serial.serial = freenect2_dev->getSerialNumber();

            enumerated_devices_.push_back(std::move(dev_with_serial));
            continue;
          }

          libusb_device_handle *dev_handle;
          r = libusb_open(dev, &dev_handle);

          if(r == LIBUSB_SUCCESS)
          {
            unsigned char buffer[1024];
            r = libusb_get_string_descriptor_ascii(dev_handle, dev_desc.iSerialNumber, buffer, sizeof(buffer));
            // keep the ref until determined not kinect
            std::unique_ptr<libusb_device, LibusbDeviceDeleter> dev_ref(dev);
            libusb_close(dev_handle);

            if(r > LIBUSB_SUCCESS)
            {
              UsbDeviceWithSerial dev_with_serial;
              dev_with_serial.dev = std::move(dev_ref);
              dev_with_serial.serial = std::string(reinterpret_cast<char *>(buffer), size_t(r));

                            LOG_INFO("found valid Kinect v2 {} with serial {}", printBusAndDevice(dev), dev_with_serial.serial);
              // valid Kinect v2
              enumerated_devices_.push_back(std::move(dev_with_serial));
            }
            else
            {
                            LOG_ERROR("failed to get serial number of Kinect v2: {}", printBusAndDevice(dev, r));
            }
            continue;
          }

                    LOG_ERROR("failed to open Kinect v2: {}", printBusAndDevice(dev, r));
        }
        libusb_unref_device(dev);
      }
    }

    libusb_free_device_list(device_list, 0);
    has_device_enumeration_ = true;

        LOG_INFO("found {} devices", enumerated_devices_.size());
  }

  int getNumDevices()
  {
    if (!initialized)
      return 0;

    if(!has_device_enumeration_)
    {
      enumerateDevices();
    }
    return enumerated_devices_.size();
  }

  Freenect2Device *openDevice(int idx, std::unique_ptr<const PacketPipeline> factory, bool attempting_reset);
};

class Freenect2ReplayImpl
{
private:
  typedef std::vector<Freenect2ReplayDevice*> DeviceVector;
  DeviceVector devices_;

public:
  ~Freenect2ReplayImpl() noexcept
  {
    clearDevices();
  }

  void addDevice(Freenect2ReplayDevice *device)
  {
    devices_.push_back(device);
  }

  void removeDevice(Freenect2ReplayDevice *device)
  {
    auto it = std::find(devices_.begin(), devices_.end(), device);

    if(it != devices_.end())
    {
      devices_.erase(it);
    }
    else
    {
            LOG_WARNING("tried to remove a REPLAY device, which is not in the internal device list!");
    }
  }

  void clearDevices()
  {
    DeviceVector devices(devices_.begin(), devices_.end());

    for(auto dev : devices)
    {
      delete dev;
    }

    // Should never happen
    if(!devices_.empty())
    {
            LOG_WARNING("after deleting all REPLAY devices the internal device list should be empty!");
    }
  }

  Freenect2Device *openDevice(const std::vector<std::string>& frame_filenames, std::unique_ptr<const PacketPipeline> pipeline);
};

Freenect2DeviceImpl::Freenect2DeviceImpl(Freenect2Impl *context, std::unique_ptr<const PacketPipeline> pipeline, libusb_device *usb_device, libusb_device_handle *usb_device_handle, const std::string &serial) :
  state_(Created),
  has_usb_interfaces_(false),
  context_(context),
  usb_device_(usb_device),
  usb_device_handle_(usb_device_handle),
  rgb_transfer_pool_(usb_device_handle, 0x83),
  ir_transfer_pool_(usb_device_handle, 0x84),
  usb_control_(usb_device_handle_.get()),
  command_tx_(usb_device_handle_.get(), 0x81, 0x02),
  command_seq_(0),
  pipeline_(std::move(pipeline)),
  serial_(serial),
  firmware_("<unknown>")
{
  rgb_transfer_pool_.setCallback(pipeline_->getRgbPacketParser());
  ir_transfer_pool_.setCallback(pipeline_->getIrPacketParser());
}

Freenect2DeviceImpl::~Freenect2DeviceImpl()
{
  close();
  context_->removeDevice(this);
}

int Freenect2DeviceImpl::nextCommandSeq()
{
  return command_seq_++;
}

bool Freenect2DeviceImpl::isSameUsbDevice(libusb_device* other)
{
  bool result = false;

  if(state_ != Closed && usb_device_ != nullptr)
  {
    unsigned char bus = libusb_get_bus_number(usb_device_.get());
    unsigned char address = libusb_get_device_address(usb_device_.get());

    unsigned char other_bus = libusb_get_bus_number(other);
    unsigned char other_address = libusb_get_device_address(other);

    result = (bus == other_bus) && (address == other_address);
  }

  return result;
}

std::string Freenect2DeviceImpl::getSerialNumber()
{
  return serial_;
}

std::string Freenect2DeviceImpl::getFirmwareVersion()
{
  return firmware_;
}

Freenect2Device::ColorCameraParams Freenect2DeviceImpl::getColorCameraParams()
{
  return rgb_camera_params_;
}


Freenect2Device::IrCameraParams Freenect2DeviceImpl::getIrCameraParams()
{
  return ir_camera_params_;
}

void Freenect2DeviceImpl::setColorCameraParams(const Freenect2Device::ColorCameraParams &params)
{
  rgb_camera_params_ = params;
}

void Freenect2DeviceImpl::setIrCameraParams(const Freenect2Device::IrCameraParams &params)
{
  ir_camera_params_ = params;
  DepthPacketProcessor *proc = pipeline_->getDepthPacketProcessor();
  if (proc != nullptr)
  {
    IrCameraTables tables(params);
    proc->loadXZTables(tables.xtable.data(), tables.ztable.data());
    proc->loadLookupTable(tables.lut.data());
  }
}

void Freenect2DeviceImpl::setConfiguration(const Freenect2Device::Config &config)
{
  DepthPacketProcessor *proc = pipeline_->getDepthPacketProcessor();
  if (proc != nullptr)
    proc->setConfiguration(config);
}

void Freenect2DeviceImpl::setColorFrameListener(libfreenect2::FrameListener* rgb_frame_listener)
{
  // TODO: should only be possible, if not started
  if(pipeline_->getRgbPacketProcessor() != nullptr)
    pipeline_->getRgbPacketProcessor()->setFrameListener(rgb_frame_listener);
}

void Freenect2DeviceImpl::setIrAndDepthFrameListener(libfreenect2::FrameListener* ir_frame_listener)
{
  // TODO: should only be possible, if not started
  if(pipeline_->getDepthPacketProcessor() != nullptr)
    pipeline_->getDepthPacketProcessor()->setFrameListener(ir_frame_listener);
}

void Freenect2DeviceImpl::setColorAutoExposure(float exposure_compensation)
{
  CommandTransaction::Result result;
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_ACS, 0u), result);
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_EXPOSURE_MODE, 0u), result);  // 0 == Fully auto
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_EXPOSURE_COMPENSATION, exposure_compensation), result);
}

void Freenect2DeviceImpl::setColorSemiAutoExposure(float pseudo_exposure_time_ms)
{
  CommandTransaction::Result result;
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_ACS, 0u), result);
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_EXPOSURE_MODE, 3u), result);  // 3 == Semi-auto
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_EXPOSURE_TIME_MS, pseudo_exposure_time_ms), result);
}

void Freenect2DeviceImpl::setColorManualExposure(float integration_time_ms, float analog_gain)
{
  CommandTransaction::Result result;
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_ACS, 0u), result);
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_EXPOSURE_MODE, 4u), result);  // 4 == Fully manual
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_INTEGRATION_TIME, integration_time_ms), result);
  command_tx_.execute(ColorSettingCommand(COLOR_SETTING_SET_ANALOG_GAIN, analog_gain), result);
}

void Freenect2DeviceImpl::setColorSetting(ColorSettingCommandType cmd, uint32_t value)
{
  CommandTransaction::Result result;
  command_tx_.execute(ColorSettingCommand(cmd, value), result);
}

void Freenect2DeviceImpl::setColorSetting(ColorSettingCommandType cmd, float value)
{
  CommandTransaction::Result result;
  command_tx_.execute(ColorSettingCommand(cmd, value), result);
}

uint32_t Freenect2DeviceImpl::getColorSetting(ColorSettingCommandType cmd)
{
  CommandTransaction::Result result;
  command_tx_.execute(ColorSettingCommand(cmd), result);
  if (result.size() < sizeof(ColorSettingResponse))
  {
        LOG_WARNING("failed to get color setting, response size {} too small, expected {}", result.size(), sizeof(ColorSettingResponse));
    return 0u;
  }
  ColorSettingResponse const* data = reinterpret_cast<ColorSettingResponse const*>(result.data());
  return data->Data;
}

float Freenect2DeviceImpl::getColorSettingFloat(ColorSettingCommandType cmd)
{
  uint32_t data = getColorSetting(cmd);
  float out;
  memcpy(&out, &data, sizeof(out));
  return out;
}

void Freenect2DeviceImpl::setLedStatus(LedSettings led)
{
  CommandTransaction::Result result;
  command_tx_.execute(LedSettingCommand(led), result);
}

bool Freenect2DeviceImpl::open()
{
    LOG_INFO("opening...");

  if(state_ != Created) return false;

  if(usb_control_.setConfiguration() != UsbControl::Success) return false;
  if(!has_usb_interfaces_ && usb_control_.claimInterfaces() != UsbControl::Success) return false;
  has_usb_interfaces_ = true;

  if(usb_control_.setIsochronousDelay() != UsbControl::Success) return false;
  // TODO: always fails right now with error 6 - TRANSFER_OVERFLOW!
  //if(usb_control_.setPowerStateLatencies() != UsbControl::Success) return false;
  if(usb_control_.setIrInterfaceState(UsbControl::Disabled) != UsbControl::Success) return false;
  if(usb_control_.enablePowerStates() != UsbControl::Success) return false;
  if(usb_control_.setVideoTransferFunctionState(UsbControl::Disabled) != UsbControl::Success) return false;

  int max_iso_packet_size;
  if(usb_control_.getIrMaxIsoPacketSize(max_iso_packet_size) != UsbControl::Success) return false;

  if(max_iso_packet_size < 0x8400)
  {
        LOG_ERROR("max iso packet size for endpoint 0x84 too small! (expected: {} got: {})", 0x8400, max_iso_packet_size);
    return false;
  }

  unsigned rgb_xfer_size = 0x4000;
  unsigned rgb_num_xfers = 20;
  unsigned ir_pkts_per_xfer = 8;
  unsigned ir_num_xfers = 60;

#if defined(__APPLE__)
  ir_pkts_per_xfer = 128;
  ir_num_xfers = 4;
#elif defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
  // For multi-Kinect setup, there is a 64 fd limit on poll().
  rgb_xfer_size = 1048576;
  rgb_num_xfers = 3;
  ir_pkts_per_xfer = 64;
  ir_num_xfers = 8;
#endif

  const char *xfer_str;
  xfer_str = std::getenv("LIBFREENECT2_RGB_TRANSFER_SIZE");
  if(xfer_str) rgb_xfer_size = std::atoi(xfer_str);
  xfer_str = std::getenv("LIBFREENECT2_RGB_TRANSFERS");
  if(xfer_str) rgb_num_xfers = std::atoi(xfer_str);
  xfer_str = std::getenv("LIBFREENECT2_IR_PACKETS");
  if(xfer_str) ir_pkts_per_xfer = std::atoi(xfer_str);
  xfer_str = std::getenv("LIBFREENECT2_IR_TRANSFERS");
  if(xfer_str) ir_num_xfers = std::atoi(xfer_str);

    LOG_INFO("transfer pool sizes rgb: {}*{} ir: {}*{}*{}", rgb_num_xfers, rgb_xfer_size, ir_num_xfers, ir_pkts_per_xfer, max_iso_packet_size);
  rgb_transfer_pool_.allocate(rgb_num_xfers, rgb_xfer_size);
  ir_transfer_pool_.allocate(ir_num_xfers, ir_pkts_per_xfer, max_iso_packet_size);

  state_ = Open;

    LOG_INFO("opened");

  return true;
}

bool Freenect2DeviceImpl::start()
{
  return startStreams(true, true);
}

bool Freenect2DeviceImpl::startStreams(bool enable_rgb, bool enable_depth)
{
    LOG_INFO("starting...");
  if(state_ != Open) return false;

  CommandTransaction::Result serial_result, firmware_result, result;

  if (usb_control_.setVideoTransferFunctionState(UsbControl::Enabled) != UsbControl::Success) return false;

  if (!command_tx_.execute(ReadFirmwareVersionsCommand(nextCommandSeq()), firmware_result)) return false;
  firmware_ = FirmwareVersionResponse(firmware_result).toString();

  if (!command_tx_.execute(ReadHardwareInfoCommand(nextCommandSeq()), result)) return false;
  //The hardware version is currently useless.  It is only used to select the
  //IR normalization table, but we don't have that.

  if (!command_tx_.execute(ReadSerialNumberCommand(nextCommandSeq()), serial_result)) return false;
  std::string new_serial = SerialNumberResponse(serial_result).toString();

  if(serial_ != new_serial)
  {
        LOG_WARNING("serial number reported by libusb {} differs from serial number {} in device protocol! ", serial_, new_serial);
  }

  if (!command_tx_.execute(ReadDepthCameraParametersCommand(nextCommandSeq()), result)) return false;
  setIrCameraParams(DepthCameraParamsResponse(result).toIrCameraParams());

  if (!command_tx_.execute(ReadP0TablesCommand(nextCommandSeq()), result)) return false;
  if(pipeline_->getDepthPacketProcessor() != nullptr)
    pipeline_->getDepthPacketProcessor()->loadP0TablesFromCommandResponse(result.data(), result.size());

  if (!command_tx_.execute(ReadRgbCameraParametersCommand(nextCommandSeq()), result)) return false;
  setColorCameraParams(RgbCameraParamsResponse(result).toColorCameraParams());

  if (!command_tx_.execute(SetModeEnabledWith0x00640064Command(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetModeDisabledCommand(nextCommandSeq()), result)) return false;

  int timeout = 50; // about 5 seconds (100ms x 50)
  for (uint32_t status = 0, last = 0; (status & 1) == 0 && 0 < timeout; last = status, timeout--)
  {
    if (!command_tx_.execute(ReadStatus0x090000Command(nextCommandSeq()), result)) return false;
    status = Status0x090000Response(result).toNumber();
    if (status != last)
            LOG_DEBUG("status 0x090000: {}", status);
    if ((status & 1) == 0)
      this_thread::sleep_for(chrono::milliseconds(100));
  }
  if (timeout == 0) {
        LOG_DEBUG("status 0x090000: timeout");
  }

  if (!command_tx_.execute(InitStreamsCommand(nextCommandSeq()), result)) return false;

  if (usb_control_.setIrInterfaceState(UsbControl::Enabled) != UsbControl::Success) return false;

  if (!command_tx_.execute(ReadStatus0x090000Command(nextCommandSeq()), result)) return false;
    LOG_DEBUG("status 0x090000: {}", Status0x090000Response(result).toNumber());

  if (!command_tx_.execute(SetStreamEnabledCommand(nextCommandSeq()), result)) return false;

  //command_tx_.execute(Unknown0x47Command(nextCommandSeq()), result);
  //command_tx_.execute(Unknown0x46Command(nextCommandSeq()), result);
/*
  command_tx_.execute(SetModeEnabledCommand(nextCommandSeq()), result);
  command_tx_.execute(SetModeDisabledCommand(nextCommandSeq()), result);

  usb_control_.setIrInterfaceState(UsbControl::Enabled);

  command_tx_.execute(SetModeEnabledWith0x00640064Command(nextCommandSeq()), result);
  command_tx_.execute(ReadData0x26Command(nextCommandSeq()), result);
  command_tx_.execute(ReadStatus0x100007Command(nextCommandSeq()), result);
  command_tx_.execute(SetModeEnabledWith0x00500050Command(nextCommandSeq()), result);
  command_tx_.execute(ReadData0x26Command(nextCommandSeq()), result);
  command_tx_.execute(ReadStatus0x100007Command(nextCommandSeq()), result);
  command_tx_.execute(ReadData0x26Command(nextCommandSeq()), result);
  command_tx_.execute(ReadData0x26Command(nextCommandSeq()), result);
*/
  if (enable_rgb)
  {
        LOG_INFO("submitting rgb transfers...");
    rgb_transfer_pool_.enableSubmission();
    if (!rgb_transfer_pool_.submit()) return false;
  }

  if (enable_depth)
  {
        LOG_INFO("submitting depth transfers...");
    ir_transfer_pool_.enableSubmission();
    if (!ir_transfer_pool_.submit()) return false;
  }

  state_ = Streaming;
    LOG_INFO("started");
  return true;
}

bool Freenect2DeviceImpl::stop()
{
    LOG_INFO("stopping...");

  if(state_ != Streaming)
  {
        LOG_INFO("already stopped, doing nothing");
    return false;
  }

  if (rgb_transfer_pool_.enabled())
  {
        LOG_INFO("canceling rgb transfers...");
    rgb_transfer_pool_.disableSubmission();
    rgb_transfer_pool_.cancel();
  }

  if (ir_transfer_pool_.enabled())
  {
        LOG_INFO("canceling depth transfers...");
    ir_transfer_pool_.disableSubmission();
    ir_transfer_pool_.cancel();
  }

  if (usb_control_.setIrInterfaceState(UsbControl::Disabled) != UsbControl::Success) return false;

  CommandTransaction::Result result;
  if (!command_tx_.execute(SetModeEnabledWith0x00640064Command(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetModeDisabledCommand(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(StopCommand(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetStreamDisabledCommand(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetModeEnabledCommand(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetModeDisabledCommand(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetModeEnabledCommand(nextCommandSeq()), result)) return false;
  if (!command_tx_.execute(SetModeDisabledCommand(nextCommandSeq()), result)) return false;

  if (usb_control_.setVideoTransferFunctionState(UsbControl::Disabled) != UsbControl::Success) return false;

  state_ = Open;
    LOG_INFO("stopped");
  return true;
}

bool Freenect2DeviceImpl::close()
{
    LOG_INFO("closing...");

  if(state_ == Closed)
  {
        LOG_INFO("already closed, doing nothing");
    return true;
  }

  if(state_ == Streaming)
  {
    stop();
  }

  CommandTransaction::Result result;
  command_tx_.execute(SetModeEnabledWith0x00640064Command(nextCommandSeq()), result);
  command_tx_.execute(SetModeDisabledCommand(nextCommandSeq()), result);
  /* This command actually reboots the device and makes it disappear for 3 seconds.
   * Protonect can restart instantly without it.
   */
#ifdef __APPLE__
  /* Kinect will disappear on Mac OS X regardless during close().
   * Painstaking effort could not determine the root cause.
   * See https://github.com/OpenKinect/libfreenect2/issues/539
   *
   * Shut down Kinect explicitly on Mac and wait a fixed time.
   */
  command_tx_.execute(ShutdownCommand(nextCommandSeq()), result);
  std::this_thread::sleep_for(std::chrono::milliseconds(4*1000));
#endif

  if(pipeline_->getRgbPacketProcessor() != nullptr)
    pipeline_->getRgbPacketProcessor()->setFrameListener(nullptr);

  if(pipeline_->getDepthPacketProcessor() != nullptr)
    pipeline_->getDepthPacketProcessor()->setFrameListener(nullptr);

  if(has_usb_interfaces_)
  {
        LOG_INFO("releasing usb interfaces...");

    usb_control_.releaseInterfaces();
    has_usb_interfaces_ = false;
  }

    LOG_INFO("deallocating usb transfer pools...");
  rgb_transfer_pool_.deallocate();
  ir_transfer_pool_.deallocate();

    LOG_INFO("closing usb device...");

  usb_device_handle_.reset();
  usb_device_.reset();

  state_ = Closed;
    LOG_INFO("closed");
  return true;
}

std::unique_ptr<const PacketPipeline> createPacketPipelineByName(std::string_view name)
{
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
  if (name == "gl")
    return std::make_unique<OpenGLPacketPipeline>();
#endif
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
  if (name == "cuda")
    return std::make_unique<CudaPacketPipeline>();
#endif
#if defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
  if (name == "cl")
    return std::make_unique<OpenCLPacketPipeline>();
#endif
  if (name == "cpu")
    return std::make_unique<CpuPacketPipeline>();
  return nullptr;
}

std::unique_ptr<const PacketPipeline> createDefaultPacketPipeline()
{
  const char *pipeline_env = std::getenv("LIBFREENECT2_PIPELINE");
  if (pipeline_env)
  {
    auto pipeline = createPacketPipelineByName(pipeline_env);
    if (pipeline)
      return pipeline;
    else
            LOG_WARNING("`{}' pipeline is not available.", pipeline_env);
  }

#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
  return std::make_unique<OpenGLPacketPipeline>();
#elif defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
  return std::make_unique<CudaPacketPipeline>();
#elif defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
  return std::make_unique<OpenCLPacketPipeline>();
#else
  return std::make_unique<CpuPacketPipeline>();
#endif
}

Freenect2::Freenect2(void *usb_context) :
    impl_(std::make_unique<Freenect2Impl>(usb_context))
{
}

Freenect2::~Freenect2()
{
}

int Freenect2::enumerateDevices()
{
  impl_->clearDeviceEnumeration();
  return impl_->getNumDevices();
}

std::string Freenect2::getDeviceSerialNumber(int idx)
{
  if (!impl_->initialized)
    return {};
  if (idx >= impl_->getNumDevices() || idx < 0)
    return {};

  return impl_->enumerated_devices_[idx].serial;
}

std::string Freenect2::getDefaultDeviceSerialNumber()
{
  return getDeviceSerialNumber(0);
}

Freenect2Device *Freenect2::openDevice(int idx)
{
  return impl_->openDevice(idx, createDefaultPacketPipeline(), true);
}

Freenect2Device *Freenect2::openDevice(int idx, const PacketPipeline *pipeline)
{
  return impl_->openDevice(idx, std::unique_ptr<const PacketPipeline>(pipeline), true);
}

Freenect2Device *Freenect2Impl::openDevice(int idx, std::unique_ptr<const PacketPipeline> pipeline, bool attempting_reset)
{
  int num_devices = getNumDevices();
  Freenect2DeviceImpl *device = nullptr;

  if(idx >= num_devices)
  {
        LOG_ERROR("requested device {} is not connected!", idx);

    return device;
  }

  Freenect2Impl::UsbDeviceWithSerial &dev = enumerated_devices_[idx];
  libusb_device_handle *dev_handle = nullptr;

  if(tryGetDevice(dev.dev.get(), &device))
  {
        LOG_WARNING("device {} is already be open!", printBusAndDevice(dev.dev.get()));

    return device;
  }

  int r;
  for (int i = 0; i < 10; i++)
  {
    r = libusb_open(dev.dev.get(), &dev_handle);
    if(r == LIBUSB_SUCCESS)
    {
      break;
    }
        LOG_INFO("device unavailable right now, retrying");
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  if(r != LIBUSB_SUCCESS)
  {
        LOG_ERROR("failed to open Kinect v2: {}", printBusAndDevice(dev.dev.get(), r));

    return device;
  }

  std::unique_ptr<libusb_device_handle, LibusbHandleDeleter> dev_handle_ptr(dev_handle);

  if(attempting_reset)
  {
    r = libusb_reset_device(dev_handle_ptr.get());

    if(r == LIBUSB_ERROR_NOT_FOUND)
    {
      // From libusb documentation:
      // "If the reset fails, the descriptors change, or the previous state
      // cannot be restored, the device will appear to be disconnected and
      // reconnected. This means that the device handle is no longer valid (you
      // should close it) and rediscover the device. A return code of
      // LIBUSB_ERROR_NOT_FOUND indicates when this is the case."

      // be a good citizen
      dev_handle_ptr.reset();

      // HACK: wait for the planets to align... (When the reset fails it may
      // take a short while for the device to show up on the bus again. In the
      // absence of hotplug support, we just wait a little. If this code path
      // is followed there will already be a delay opening the device fully so
      // adding a little more is tolerable.)
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // reenumerate devices
            LOG_INFO("re-enumerating devices after reset");
      clearDeviceEnumeration();
      enumerateDevices();

      // re-open without reset
      return openDevice(idx, std::move(pipeline), false);
    }
    else if(r != LIBUSB_SUCCESS)
    {
            LOG_ERROR("failed to reset Kinect v2: {}", printBusAndDevice(dev.dev.get(), r));

      return device;
    }
  }

  // the device owns its own device ref, independent of the enumeration list
  libusb_ref_device(dev.dev.get());
  device = new Freenect2DeviceImpl(this, std::move(pipeline), dev.dev.get(), dev_handle_ptr.release(), dev.serial);
  addDevice(device);

  if(!device->open())
  {
    delete device;
    device = nullptr;

        LOG_ERROR("failed to open Kinect v2: {}", printBusAndDevice(dev.dev.get()));
  }

  return device;
}

Freenect2Device *Freenect2::openDevice(const std::string &serial)
{
  std::unique_ptr<const PacketPipeline> pipeline = createDefaultPacketPipeline();

  int num_devices = impl_->getNumDevices();
  for(int idx = 0; idx < num_devices; ++idx)
  {
    if(impl_->enumerated_devices_[idx].serial == serial)
    {
      return impl_->openDevice(idx, std::move(pipeline), true);
    }
  }

  return nullptr;
}

Freenect2Device *Freenect2::openDevice(const std::string &serial, const PacketPipeline *pipeline)
{
  std::unique_ptr<const PacketPipeline> p(pipeline);

  int num_devices = impl_->getNumDevices();
  for(int idx = 0; idx < num_devices; ++idx)
  {
    if(impl_->enumerated_devices_[idx].serial == serial)
    {
      return impl_->openDevice(idx, std::move(p), true);
    }
  }

  return nullptr;
}

Freenect2Device *Freenect2::openDefaultDevice()
{
  return openDevice(0);
}

Freenect2Device *Freenect2::openDefaultDevice(const PacketPipeline *pipeline)
{
  return openDevice(0, pipeline);
}

Freenect2ReplayDevice::Freenect2ReplayDevice(Freenect2ReplayImpl *context, const std::vector<std::string>& frame_filenames, std::unique_ptr<const PacketPipeline> pipeline)
  :context_(context), pipeline_(std::move(pipeline)), frame_filenames_(frame_filenames), running_(false)
{
  size_t single_image = 512*424*11/8;
  buffer_size_ = 10 * single_image;
  pipeline_->getDepthPacketProcessor()->allocateBuffer(packet_, buffer_size_);
}

Freenect2ReplayDevice::~Freenect2ReplayDevice()
{
  close();
  context_->removeDevice(this);
}

std::string Freenect2ReplayDevice::getSerialNumber()
{
  // Reasonable assumption given it is a software serial for apps that display this
  return LIBFREENECT2_VERSION;
}

std::string Freenect2ReplayDevice::getFirmwareVersion()
{
  // Reasonable assumption given it is a software serial for apps that display this
  return LIBFREENECT2_VERSION;
}

Freenect2Device::ColorCameraParams Freenect2ReplayDevice::getColorCameraParams()
{
  return rgb_camera_params_;
}

Freenect2Device::IrCameraParams Freenect2ReplayDevice::getIrCameraParams()
{
  return ir_camera_params_;
}

void Freenect2ReplayDevice::setColorCameraParams(const Freenect2Device::ColorCameraParams &params)
{
  rgb_camera_params_ = params;
}

void Freenect2ReplayDevice::setIrCameraParams(const Freenect2Device::IrCameraParams &params)
{
  ir_camera_params_ = params;
  DepthPacketProcessor *proc = pipeline_->getDepthPacketProcessor();
  if (proc != nullptr)
  {
    IrCameraTables tables(params);
    proc->loadXZTables(tables.xtable.data(), tables.ztable.data());
    proc->loadLookupTable(tables.lut.data());
  }
}

void Freenect2ReplayDevice::setConfiguration(const Freenect2Device::Config &config)
{
  DepthPacketProcessor *proc = pipeline_->getDepthPacketProcessor();
  if (proc != nullptr)
    proc->setConfiguration(config);
}

void Freenect2ReplayDevice::setColorFrameListener(FrameListener* listener)
{
  RgbPacketProcessor* proc = pipeline_->getRgbPacketProcessor();
  if (proc != nullptr)
  {
    proc->setFrameListener(listener);
  }
}

void Freenect2ReplayDevice::setIrAndDepthFrameListener(FrameListener* listener)
{
  DepthPacketProcessor* proc = pipeline_->getDepthPacketProcessor();
  if (proc != nullptr)
  {
    proc->setFrameListener(listener);
  }
}

bool Freenect2ReplayDevice::open()
{
    LOG_INFO("opening...");

  // May add file checking or params loading

  return true;
}

bool Freenect2ReplayDevice::close()
{
    LOG_INFO("closing...");

  if(running_ == false)
  {
        LOG_INFO("already closed, doing nothing");
    return true;
  }

  if(running_ == true)
  {
    stop();
  }

  if(pipeline_->getRgbPacketProcessor() != nullptr)
    pipeline_->getRgbPacketProcessor()->setFrameListener(nullptr);

  if(pipeline_->getDepthPacketProcessor() != nullptr)
    pipeline_->getDepthPacketProcessor()->setFrameListener(nullptr);

  running_ = false;
    LOG_INFO("closed");
  return true;
}

bool Freenect2ReplayDevice::processRawFrame(Frame::Type type, Frame* frame)
{
  if (frame->format != Frame::Raw)
  {
    return false;
  }
  switch (type)
  {
  case Frame::Color:
    processRgbFrame(frame);
    break;
  case Frame::Depth:
    processDepthFrame(frame);
    break;
  default:
    return false;
  }
  return true;
}

void Freenect2ReplayDevice::processRgbFrame(Frame* frame)
{
  RgbPacket packet;
  
  packet.timestamp = frame->timestamp;
  packet.sequence = frame->sequence;
  packet.jpeg_buffer = frame->data;
  packet.jpeg_buffer_length = frame->bytes_per_pixel;
  packet.exposure = frame->exposure;
  packet.gain = frame->gain;
  packet.gamma = frame->gamma;

  pipeline_->getRgbPacketProcessor()->process(packet);
}

void Freenect2ReplayDevice::processDepthFrame(Frame* frame)
{
  DepthPacket packet;

  packet.timestamp = frame->timestamp;
  packet.sequence = frame->sequence;
  packet.buffer = frame->data;
  packet.buffer_length = frame->bytes_per_pixel;
 
  pipeline_->getDepthPacketProcessor()->process(packet);
}

void Freenect2ReplayDevice::loadP0Tables(unsigned char* buffer, size_t buffer_length)
{
  pipeline_->getDepthPacketProcessor()->loadP0TablesFromCommandResponse(buffer, buffer_length);
}

void Freenect2ReplayDevice::static_execute(void* arg)
{
  static_cast<Freenect2ReplayDevice*>(arg)->run();
}

bool Freenect2ReplayDevice::start()
{
  running_ = true;
  t_ = std::make_unique<std::thread>(static_execute, this);
    LOG_INFO("replay started");
  return running_;
}

bool Freenect2ReplayDevice::startStreams(bool enable_rgb, bool enable_depth)
{
    LOG_INFO("Freenect2ReplayDevice: starting: rgb: {}, depth: {}", enable_rgb, enable_depth);
    LOG_INFO("Freenect2ReplayDevice: unimplemented");
  return false;
}

bool Freenect2ReplayDevice::stop()
{
  running_ = false;
  t_->join();
  t_.reset();
    LOG_INFO("replay stopped");
  return true;
}

bool hasSuffix(const std::string& str, const std::string& suffix)
{
  if (str.length() < suffix.length())
  {
    return false;
  }
  return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

bool parseFrameFilename(const std::string& frame_filename, size_t timestamp_sequence[2])
{
    LOG_DEBUG("parsing: {}", frame_filename);
  
  if (!hasSuffix(frame_filename, ".depth") &&
    !hasSuffix(frame_filename, ".jpg") &&
    !hasSuffix(frame_filename, ".jpeg"))
  {
        LOG_ERROR("wrong suffix in the filename; need .depth, .jpg, or .jpeg");
    return false;
  }
  
  size_t ix1 = frame_filename.find('_');
  size_t ix2 = frame_filename.find('_', ix1 + 1);
  size_t ix3 = frame_filename.find('.', ix2 + 1);

  std::string ts = frame_filename.substr(0, ix1);
  std::string seq = frame_filename.substr(ix2 + 1, ix3);

    LOG_DEBUG("ts: {}, seq: {}", ts, seq);

  if(ts.empty() || seq.empty())
  {
        LOG_ERROR("could not extract timestamp or sequence");
    return false;
  }
  
  timestamp_sequence[0] = atoi(ts.c_str());
  timestamp_sequence[1] = atoi(seq.c_str());

    LOG_DEBUG("ts: {}, seq: {}", timestamp_sequence[0], timestamp_sequence[1]);

  if(timestamp_sequence[0] == 0)
  {
        LOG_WARNING("invalid timestamp");
    return false;
  }
  
  return true;
}

void Freenect2ReplayDevice::run()
{
  size_t timestamp_sequence[2] = {0};
    
  for (size_t i = 0; i < frame_filenames_.size() && running_; i++)
  {
    std::string frame = frame_filenames_[i];

    if(parseFrameFilename(frame, timestamp_sequence) == false)
    {
            LOG_ERROR("could not parse replay frame filename {}, skipping...", frame);
      continue;
    }

    if (hasSuffix(frame, ".depth"))
    {
      std::ifstream fd(frame.c_str());
      
      if(!fd)
      {
                LOG_ERROR("failed to open replay frame: {}, skipping...", frame);
        continue;
      }

      fd.seekg(0, std::ifstream::end);
      size_t length = fd.tellg();
      fd.seekg(0, std::ifstream::beg);

      if(length != buffer_size_)
      {
                LOG_ERROR("file length: {}exceeds depth image buffer size: {}; skipping...", length, buffer_size_);
        continue;
      }

      fd.read(reinterpret_cast<char*>(packet_.memory->data), length);
      if(!fd || (size_t)fd.gcount() != length)
      {
                LOG_ERROR("failed to read replay frame: {}: {} vs. {} bytes", frame, fd.gcount(), length);
        continue;
      }

      if(pipeline_->getDepthPacketProcessor()->ready())
      {
        packet_.timestamp = timestamp_sequence[0];
        packet_.sequence = timestamp_sequence[1];
        packet_.buffer = packet_.memory->data;
        packet_.buffer_length = length;

        pipeline_->getDepthPacketProcessor()->process(packet_);
        pipeline_->getDepthPacketProcessor()->allocateBuffer(packet_, buffer_size_);
      }
      else
      {
                LOG_DEBUG("skipping a replay depth packet for {} as depth processor is not ready", frame);
      }
    }
  }
}

Freenect2Replay::Freenect2Replay() :
    impl_(std::make_unique<Freenect2ReplayImpl>())
{
}

Freenect2Replay::~Freenect2Replay()
{
}

Freenect2Device *Freenect2Replay::openDevice(const std::vector<std::string>& frame_filenames)
{
  return impl_->openDevice(frame_filenames, createDefaultPacketPipeline());
}

Freenect2Device *Freenect2Replay::openDevice(const std::vector<std::string>& frame_filenames, const PacketPipeline *pipeline)
{
  return impl_->openDevice(frame_filenames, std::unique_ptr<const PacketPipeline>(pipeline));
}

Freenect2Device *Freenect2ReplayImpl::openDevice(const std::vector<std::string>& frame_filenames, std::unique_ptr<const PacketPipeline> pipeline)
{
  Freenect2ReplayDevice *device = new Freenect2ReplayDevice(this, frame_filenames, std::move(pipeline));
  addDevice(device);

  if(!device->open())
  {
    delete device;
    device = nullptr;
        LOG_ERROR("failed to instantiate a replay device!");
  }

  return device;
}

} /* namespace libfreenect2 */
