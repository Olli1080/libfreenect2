// src/python/bind_registration.cpp — Registration.
//
// v1 skips two niche pieces of the C++ API (documented, not silently
// dropped): the raw `color_depth_map` int* out-param of apply() (a 512x424
// index buffer with no natural Python-side counterpart without adding a
// second numpy-shaped output convention just for this one field), and
// DumpPacketPipeline-style raw table access (irrelevant here, see
// bind_pipeline.cpp). The scalar apply(dx, dy, dz) -> (cx, cy) point
// overload and the Frame* overload are both bound, matching the two
// overloads in registration.h.

#include "bindings.h"

#include <libfreenect2/registration.h>
#include <libfreenect2/frame_listener.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

namespace
{

py::tuple applyPoint(const Registration &self, int dx, int dy, float dz)
{
  float cx = 0.f, cy = 0.f;
  self.apply(dx, dy, dz, cx, cy);
  return py::make_tuple(cx, cy);
}

// Registration::apply()/undistortDepth() (registration.cpp) only ever write
// through undistorted/registered/bigdepth's raw ->data pointer -- they never
// touch ->format, which stays Frame::Invalid from construction (frame.cpp).
// C++ callers don't need it: they know statically which output params are
// float vs. byte data and cast ->data directly (e.g. Protonect-style
// `cv::Mat(h, w, CV_32FC1, bigdepth.data)`). But pyfreenect2's buffer
// protocol (bind_frame.cpp) picks numpy shape/dtype FROM Frame::format at
// runtime, and undistorted/registered are structurally indistinguishable
// otherwise (both 512x424x4) despite one being float32 and the other BGRX
// uint8 -- so it has to be set here, right where the ambiguity is created,
// from each output param's fixed, documented semantics.
void applyFrames(const Registration &self, const Frame *rgb, const Frame *depth,
                  Frame *undistorted, Frame *registered, bool enable_filter, Frame *bigdepth)
{
  {
    py::gil_scoped_release release;
    self.apply(rgb, depth, undistorted, registered, enable_filter, bigdepth, nullptr);
  }
  if (undistorted)
    undistorted->format = Frame::Float;
  if (registered)
    registered->format = rgb ? rgb->format : Frame::BGRX;
  if (bigdepth)
    bigdepth->format = Frame::Float;
}

void undistortDepthWrapper(const Registration &self, const Frame *depth, Frame *undistorted)
{
  {
    py::gil_scoped_release release;
    self.undistortDepth(depth, undistorted);
  }
  if (undistorted)
    undistorted->format = Frame::Float;
}

py::tuple getPointXYZ(const Registration &self, const Frame *undistorted, int r, int c)
{
  float x = 0.f, y = 0.f, z = 0.f;
  self.getPointXYZ(undistorted, r, c, x, y, z);
  return py::make_tuple(x, y, z);
}

py::tuple getPointXYZRGB(const Registration &self, const Frame *undistorted, const Frame *registered, int r, int c)
{
  float x = 0.f, y = 0.f, z = 0.f, rgb = 0.f;
  self.getPointXYZRGB(undistorted, registered, r, c, x, y, z, rgb);
  return py::make_tuple(x, y, z, rgb);
}

} // namespace

void bindRegistration(py::module_ &m)
{
  py::class_<Registration>(m, "Registration")
    .def(py::init<Freenect2Device::IrCameraParams, Freenect2Device::ColorCameraParams>(),
         py::arg("depth_params"), py::arg("rgb_params"))
    .def("apply_point", &applyPoint, py::arg("dx"), py::arg("dy"), py::arg("dz"),
         "Undistort/register a single distorted depth pixel (dx, dy, dz in "
         "millimeters). Returns (cx, cy) in normalized color coordinates.")
    .def("apply", &applyFrames, py::arg("rgb"), py::arg("depth"), py::arg("undistorted"),
         py::arg("registered"), py::arg("enable_filter") = true, py::arg("bigdepth") = nullptr,
         "Map color onto depth. `undistorted`/`registered` must be "
         "pre-allocated Frame(512, 424, 4); `bigdepth`, if given, must be "
         "Frame(1920, 1082, 4) and receives the depth-onto-color mapping.")
    .def("undistort_depth", &undistortDepthWrapper, py::arg("depth"), py::arg("undistorted"))
    .def("get_point_xyz", &getPointXYZ, py::arg("undistorted"), py::arg("r"), py::arg("c"),
         "Returns (x, y, z) in meters for depth pixel (r, c).")
    .def("get_point_xyz_rgb", &getPointXYZRGB, py::arg("undistorted"), py::arg("registered"),
         py::arg("r"), py::arg("c"),
         "Returns (x, y, z, rgb_packed) for depth pixel (r, c). Unpack "
         "rgb_packed's bytes as (b, g, r, _).");
}

} // namespace pyfreenect2
