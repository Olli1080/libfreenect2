// src/python/bind_frame.cpp — Frame::Type/Format, Frame, FrameListener,
// SyncMultiFrameListener.
//
// Frame lifetime has two distinct origins that must be handled differently:
//  - Frames delivered via SyncMultiFrameListener::waitForNewFrame() are
//    owned by libfreenect2 internals and freed exactly once by
//    SyncMultiFrameListener::release() -- Python must never delete these.
//    Returned with return_value_policy::reference so the Python wrapper
//    never takes ownership.
//  - Frames constructed by the caller (e.g. Frame(512, 424, 4) used as an
//    `undistorted`/`registered`/`bigdepth` output argument to
//    Registration::apply()) are normal, independently owned objects --
//    Python should own and delete these normally, so the class itself keeps
//    pybind11's default unique_ptr holder.
// No py::nodelete is used anywhere; the return policy alone is what decides
// ownership per call site, matching the C++ API's own ownership rules.

#include "bindings.h"

#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/frame_listener_impl.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

namespace
{

// Frame::data layout depends on Frame::format:
//  - Float (Ir/Depth): one 4-byte float per pixel -> (height, width) float32.
//  - Raw: bytes_per_pixel is repurposed to mean "total buffer size", not a
//    per-pixel count (see frame_listener.hpp) -> flat 1-D byte buffer.
//  - Everything else (BGRX/RGBX/Gray/Invalid): byte-per-channel data ->
//    (height, width, bytes_per_pixel) uint8, e.g. (1080, 1920, 4) for Color.
py::buffer_info frameBuffer(Frame &frame)
{
  if (frame.format == Frame::Raw)
  {
    return py::buffer_info(
      frame.data,
      1,
      py::format_descriptor<unsigned char>::format(),
      1,
      { static_cast<py::ssize_t>(frame.bytes_per_pixel) },
      { static_cast<py::ssize_t>(1) }
    );
  }

  if (frame.format == Frame::Float)
  {
    return py::buffer_info(
      frame.data,
      sizeof(float),
      py::format_descriptor<float>::format(),
      2,
      { static_cast<py::ssize_t>(frame.height), static_cast<py::ssize_t>(frame.width) },
      { static_cast<py::ssize_t>(frame.width * sizeof(float)), static_cast<py::ssize_t>(sizeof(float)) }
    );
  }

  return py::buffer_info(
    frame.data,
    1,
    py::format_descriptor<unsigned char>::format(),
    3,
    { static_cast<py::ssize_t>(frame.height), static_cast<py::ssize_t>(frame.width), static_cast<py::ssize_t>(frame.bytes_per_pixel) },
    {
      static_cast<py::ssize_t>(frame.width * frame.bytes_per_pixel),
      static_cast<py::ssize_t>(frame.bytes_per_pixel),
      static_cast<py::ssize_t>(1)
    }
  );
}

py::dict waitForNewFrame(SyncMultiFrameListener &self, int timeout_ms)
{
  FrameMap frames;
  bool ok = true;
  {
    py::gil_scoped_release release;
    if (timeout_ms >= 0)
      ok = self.waitForNewFrame(frames, timeout_ms);
    else
      self.waitForNewFrame(frames);
  }

  if (!ok)
    throw std::runtime_error("timed out waiting for a new frame");

  py::dict result;
  for (auto &kv : frames)
    result[py::cast(kv.first)] = py::cast(kv.second, py::return_value_policy::reference);
  return result;
}

void releaseFrames(SyncMultiFrameListener &self, py::dict frames)
{
  FrameMap fm;
  for (auto item : frames)
    fm[item.first.cast<Frame::Type>()] = item.second.cast<Frame *>();
  self.release(fm);
}

} // namespace

void bindFrame(py::module_ &m)
{
  // arithmetic(): frame_types is a bitmask (see SyncMultiFrameListener's
  // ctor doc, "Use bitwise or to combine multiple types") -- without this,
  // pybind11 enums don't support `|` from Python.
  py::enum_<Frame::Type>(m, "FrameType", py::arithmetic())
    .value("Color", Frame::Color)
    .value("Ir", Frame::Ir)
    .value("Depth", Frame::Depth);

  py::enum_<Frame::Format>(m, "FrameFormat")
    .value("Invalid", Frame::Invalid)
    .value("Raw", Frame::Raw)
    .value("Float", Frame::Float)
    .value("BGRX", Frame::BGRX)
    .value("RGBX", Frame::RGBX)
    .value("Gray", Frame::Gray);

  py::class_<Frame>(m, "Frame", py::buffer_protocol(),
      "width x height x bytes_per_pixel frame buffer. Supports the buffer "
      "protocol -- use numpy.asarray(frame) for a zero-copy view (valid "
      "only until the frame is released/destroyed).")
    .def(py::init<size_t, size_t, size_t>(), py::arg("width"), py::arg("height"), py::arg("bytes_per_pixel"),
         "Allocate a new, independently-owned frame buffer -- for use as an "
         "output argument to Registration.apply()/undistort_depth() "
         "(e.g. Frame(512, 424, 4) for `undistorted`/`registered`).")
    .def_readonly("width", &Frame::width)
    .def_readonly("height", &Frame::height)
    .def_readonly("bytes_per_pixel", &Frame::bytes_per_pixel)
    .def_readonly("timestamp", &Frame::timestamp)
    .def_readonly("sequence", &Frame::sequence)
    .def_readonly("exposure", &Frame::exposure)
    .def_readonly("gain", &Frame::gain)
    .def_readonly("gamma", &Frame::gamma)
    .def_readonly("status", &Frame::status)
    .def_readonly("format", &Frame::format)
    .def_buffer(&frameBuffer);

  py::class_<FrameListener>(m, "FrameListener",
      "Base callback interface. Not subclassable from Python in v1 -- use "
      "SyncMultiFrameListener instead.");

  py::class_<SyncMultiFrameListener, FrameListener>(m, "SyncMultiFrameListener")
    .def(py::init<unsigned int>(), py::arg("frame_types"),
         "frame_types: bitwise-or of FrameType values, e.g. FrameType.Ir | FrameType.Depth.")
    .def("has_new_frame", &SyncMultiFrameListener::hasNewFrame)
    .def("wait_for_new_frame", &waitForNewFrame, py::arg("timeout_ms") = -1,
         "Block for up to timeout_ms (default: forever) and return a dict "
         "of FrameType -> Frame. Raises TimeoutError-compatible RuntimeError "
         "if timeout_ms >= 0 and no frame arrived in time. Caller must pass "
         "the returned dict to release() when done with it.")
    .def("release", &releaseFrames, py::arg("frames"),
         "Free the frames in a dict previously returned by wait_for_new_frame().");
}

} // namespace pyfreenect2
