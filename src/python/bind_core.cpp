// src/python/bind_core.cpp — Freenect2, Freenect2Device, Freenect2Replay,
// ColorCameraParams/IrCameraParams, DepthProcessorConfig, LedSettings.
//
// Freenect2Device is never constructed from Python -- only obtained via
// Freenect2::open*()/Freenect2Replay::open_device(). Its lifetime is owned
// by the context (Freenect2Impl) that created it, not transferred to the
// caller (matches the C++ API: Protonect never `delete`s the returned
// pointer). openDevice()/openDefaultDevice() therefore use
// return_value_policy::reference_internal, which both marks the returned
// wrapper as non-owning and keeps the parent Freenect2/Freenect2Replay
// object alive for as long as the returned device handle exists Python-side.
//
// The PacketPipeline* overloads take std::unique_ptr<PacketPipeline> by
// value: pybind11 disowns the Python-side pipeline object when the call is
// made, matching the C++ API's "libfreenect2 will take care of it; do not
// use or free the object afterward" contract. Do not reuse a pipeline
// object across open_device() calls -- the C++ API requires a fresh one
// each time, and disowning enforces that from the Python side too.

#include "bindings.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

namespace
{

Freenect2Device *openDeviceIdx(Freenect2 &self, int idx)
{
  py::gil_scoped_release release;
  return self.openDevice(idx);
}

Freenect2Device *openDeviceIdxPipeline(Freenect2 &self, int idx, std::unique_ptr<PacketPipeline> pipeline)
{
  py::gil_scoped_release release;
  return self.openDevice(idx, pipeline.release());
}

Freenect2Device *openDeviceSerial(Freenect2 &self, const std::string &serial)
{
  py::gil_scoped_release release;
  return self.openDevice(serial);
}

Freenect2Device *openDeviceSerialPipeline(Freenect2 &self, const std::string &serial, std::unique_ptr<PacketPipeline> pipeline)
{
  py::gil_scoped_release release;
  return self.openDevice(serial, pipeline.release());
}

Freenect2Device *openDefaultDeviceNoPipeline(Freenect2 &self)
{
  py::gil_scoped_release release;
  return self.openDefaultDevice();
}

Freenect2Device *openDefaultDevicePipeline(Freenect2 &self, std::unique_ptr<PacketPipeline> pipeline)
{
  py::gil_scoped_release release;
  return self.openDefaultDevice(pipeline.release());
}

Freenect2Device *openReplayDevice(Freenect2Replay &self, const std::vector<std::string> &filenames)
{
  py::gil_scoped_release release;
  return self.openDevice(filenames);
}

Freenect2Device *openReplayDevicePipeline(Freenect2Replay &self, const std::vector<std::string> &filenames, std::unique_ptr<PacketPipeline> pipeline)
{
  py::gil_scoped_release release;
  return self.openDevice(filenames, pipeline.release());
}

bool deviceStart(Freenect2Device &self)
{
  py::gil_scoped_release release;
  return self.start();
}

bool deviceStartStreams(Freenect2Device &self, bool rgb, bool depth)
{
  py::gil_scoped_release release;
  return self.startStreams(rgb, depth);
}

bool deviceStop(Freenect2Device &self)
{
  py::gil_scoped_release release;
  return self.stop();
}

bool deviceClose(Freenect2Device &self)
{
  py::gil_scoped_release release;
  return self.close();
}

} // namespace

void bindCore(py::module_ &m)
{
  py::class_<DepthProcessorConfig>(m, "DepthProcessorConfig")
    .def(py::init<>())
    .def_readwrite("min_depth", &DepthProcessorConfig::MinDepth)
    .def_readwrite("max_depth", &DepthProcessorConfig::MaxDepth)
    .def_readwrite("enable_bilateral_filter", &DepthProcessorConfig::EnableBilateralFilter)
    .def_readwrite("enable_edge_aware_filter", &DepthProcessorConfig::EnableEdgeAwareFilter);

  py::class_<LedSettings>(m, "LedSettings")
    .def(py::init<>())
    .def_readwrite("led_id", &LedSettings::LedId)
    .def_readwrite("mode", &LedSettings::Mode)
    .def_readwrite("start_level", &LedSettings::StartLevel)
    .def_readwrite("stop_level", &LedSettings::StopLevel)
    .def_readwrite("interval_ms", &LedSettings::IntervalInMs);

  py::class_<Freenect2Device::ColorCameraParams>(m, "ColorCameraParams")
    .def(py::init<>())
    .def_readwrite("fx", &Freenect2Device::ColorCameraParams::fx)
    .def_readwrite("fy", &Freenect2Device::ColorCameraParams::fy)
    .def_readwrite("cx", &Freenect2Device::ColorCameraParams::cx)
    .def_readwrite("cy", &Freenect2Device::ColorCameraParams::cy)
    .def_readwrite("shift_d", &Freenect2Device::ColorCameraParams::shift_d)
    .def_readwrite("shift_m", &Freenect2Device::ColorCameraParams::shift_m)
    .def_readwrite("mx_x3y0", &Freenect2Device::ColorCameraParams::mx_x3y0)
    .def_readwrite("mx_x0y3", &Freenect2Device::ColorCameraParams::mx_x0y3)
    .def_readwrite("mx_x2y1", &Freenect2Device::ColorCameraParams::mx_x2y1)
    .def_readwrite("mx_x1y2", &Freenect2Device::ColorCameraParams::mx_x1y2)
    .def_readwrite("mx_x2y0", &Freenect2Device::ColorCameraParams::mx_x2y0)
    .def_readwrite("mx_x0y2", &Freenect2Device::ColorCameraParams::mx_x0y2)
    .def_readwrite("mx_x1y1", &Freenect2Device::ColorCameraParams::mx_x1y1)
    .def_readwrite("mx_x1y0", &Freenect2Device::ColorCameraParams::mx_x1y0)
    .def_readwrite("mx_x0y1", &Freenect2Device::ColorCameraParams::mx_x0y1)
    .def_readwrite("mx_x0y0", &Freenect2Device::ColorCameraParams::mx_x0y0)
    .def_readwrite("my_x3y0", &Freenect2Device::ColorCameraParams::my_x3y0)
    .def_readwrite("my_x0y3", &Freenect2Device::ColorCameraParams::my_x0y3)
    .def_readwrite("my_x2y1", &Freenect2Device::ColorCameraParams::my_x2y1)
    .def_readwrite("my_x1y2", &Freenect2Device::ColorCameraParams::my_x1y2)
    .def_readwrite("my_x2y0", &Freenect2Device::ColorCameraParams::my_x2y0)
    .def_readwrite("my_x0y2", &Freenect2Device::ColorCameraParams::my_x0y2)
    .def_readwrite("my_x1y1", &Freenect2Device::ColorCameraParams::my_x1y1)
    .def_readwrite("my_x1y0", &Freenect2Device::ColorCameraParams::my_x1y0)
    .def_readwrite("my_x0y1", &Freenect2Device::ColorCameraParams::my_x0y1)
    .def_readwrite("my_x0y0", &Freenect2Device::ColorCameraParams::my_x0y0);

  py::class_<Freenect2Device::IrCameraParams>(m, "IrCameraParams")
    .def(py::init<>())
    .def_readwrite("fx", &Freenect2Device::IrCameraParams::fx)
    .def_readwrite("fy", &Freenect2Device::IrCameraParams::fy)
    .def_readwrite("cx", &Freenect2Device::IrCameraParams::cx)
    .def_readwrite("cy", &Freenect2Device::IrCameraParams::cy)
    .def_readwrite("k1", &Freenect2Device::IrCameraParams::k1)
    .def_readwrite("k2", &Freenect2Device::IrCameraParams::k2)
    .def_readwrite("k3", &Freenect2Device::IrCameraParams::k3)
    .def_readwrite("p1", &Freenect2Device::IrCameraParams::p1)
    .def_readwrite("p2", &Freenect2Device::IrCameraParams::p2);

  py::class_<Freenect2Device>(m, "Freenect2Device",
      "Handle to an open Kinect v2 (or replay) device. Obtained from "
      "Freenect2.open_device()/open_default_device() or "
      "Freenect2Replay.open_device() -- never constructed directly. Owned "
      "by the context that created it; do not use after that context is "
      "garbage-collected.")
    .def("get_serial_number", &Freenect2Device::getSerialNumber)
    .def("get_firmware_version", &Freenect2Device::getFirmwareVersion)
    .def("get_color_camera_params", &Freenect2Device::getColorCameraParams)
    .def("get_ir_camera_params", &Freenect2Device::getIrCameraParams)
    .def("set_color_camera_params", &Freenect2Device::setColorCameraParams, py::arg("params"))
    .def("set_ir_camera_params", &Freenect2Device::setIrCameraParams, py::arg("params"))
    .def("set_configuration", &Freenect2Device::setConfiguration, py::arg("config"))
    .def("set_color_frame_listener", &Freenect2Device::setColorFrameListener, py::arg("listener"),
         py::keep_alive<1, 2>())
    .def("set_ir_and_depth_frame_listener", &Freenect2Device::setIrAndDepthFrameListener, py::arg("listener"),
         py::keep_alive<1, 2>())
    .def("set_color_auto_exposure", &Freenect2Device::setColorAutoExposure, py::arg("exposure_compensation") = 0.0f)
    .def("set_color_semi_auto_exposure", &Freenect2Device::setColorSemiAutoExposure, py::arg("pseudo_exposure_time_ms"))
    .def("set_color_manual_exposure", &Freenect2Device::setColorManualExposure,
         py::arg("integration_time_ms"), py::arg("analog_gain"))
    .def("set_color_setting", py::overload_cast<ColorSettingCommandType, uint32_t>(&Freenect2Device::setColorSetting),
         py::arg("cmd"), py::arg("value"))
    .def("set_color_setting", py::overload_cast<ColorSettingCommandType, float>(&Freenect2Device::setColorSetting),
         py::arg("cmd"), py::arg("value"))
    .def("get_color_setting", &Freenect2Device::getColorSetting, py::arg("cmd"))
    .def("get_color_setting_float", &Freenect2Device::getColorSettingFloat, py::arg("cmd"))
    .def("set_led_status", &Freenect2Device::setLedStatus, py::arg("led"))
    .def("start", &deviceStart, "Start both RGB and depth streams. Returns False on failure.")
    .def("start_streams", &deviceStartStreams, py::arg("rgb"), py::arg("depth"))
    .def("stop", &deviceStop)
    .def("close", &deviceClose);

  py::class_<Freenect2>(m, "Freenect2")
    .def(py::init<>())
    .def("enumerate_devices", &Freenect2::enumerateDevices,
         "Must be called before anything else. Returns the number of devices found.")
    .def("get_device_serial_number", &Freenect2::getDeviceSerialNumber, py::arg("idx"))
    .def("get_default_device_serial_number", &Freenect2::getDefaultDeviceSerialNumber)
    .def("open_device", &openDeviceIdx, py::arg("idx"), py::return_value_policy::reference_internal)
    .def("open_device", &openDeviceIdxPipeline, py::arg("idx"), py::arg("pipeline"),
         py::return_value_policy::reference_internal)
    .def("open_device", &openDeviceSerial, py::arg("serial"), py::return_value_policy::reference_internal)
    .def("open_device", &openDeviceSerialPipeline, py::arg("serial"), py::arg("pipeline"),
         py::return_value_policy::reference_internal)
    .def("open_default_device", &openDefaultDeviceNoPipeline, py::return_value_policy::reference_internal)
    .def("open_default_device", &openDefaultDevicePipeline, py::arg("pipeline"),
         py::return_value_policy::reference_internal);

  py::class_<Freenect2Replay>(m, "Freenect2Replay")
    .def(py::init<>())
    .def("open_device", &openReplayDevice, py::arg("frame_filenames"),
         py::return_value_policy::reference_internal)
    .def("open_device", &openReplayDevicePipeline, py::arg("frame_filenames"), py::arg("pipeline"),
         py::return_value_policy::reference_internal);
}

} // namespace pyfreenect2
