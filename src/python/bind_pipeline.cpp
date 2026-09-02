// src/python/bind_pipeline.cpp — PacketPipeline hierarchy.
//
// Each subclass is only registered under the same LIBFREENECT2_WITH_*_SUPPORT
// guard the C++ header itself uses, so pyfreenect2 only exposes backends
// actually compiled into this build; available_pipelines() lets Python
// introspect that instead of guessing from platform/config.
//
// DumpPacketPipeline is intentionally not bound: its extra methods
// (getDepthP0Tables/getDepthXTable/...) return raw calibration-table
// pointers meant for internal/dump tooling, not the common openDevice()
// flow -- out of scope for v1.
//
// PacketPipeline objects are handed to Freenect2::openDevice() (bind_core.cpp)
// via std::unique_ptr<PacketPipeline>, which pybind11 uses to transfer
// ownership out of the Python wrapper -- matching the C++ API's documented
// "libfreenect2 will take care [of deleting it]; a new PacketPipeline object
// has to be created each time" contract. This disowning transfer requires
// the whole hierarchy to use pybind11's `smart_holder` holder (confirmed by
// an actual build: pybind11 3.x's default unique_ptr holder rejects it at
// runtime with "Passing std::unique_ptr<T> from Python to C++ requires
// py::class_<T, py::smart_holder>").

#include "bindings.h"

#include <libfreenect2/config.h>
#include <libfreenect2/packet_pipeline.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>
#include <string>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

void bindPipeline(py::module_ &m)
{
  py::class_<PacketPipeline, py::smart_holder>(m, "PacketPipeline",
      "Base class of the pipeline hierarchy. Not constructible from Python "
      "-- construct one of the concrete backends below and pass it to "
      "Freenect2.open_device()/open_default_device().");

  py::class_<CpuPacketPipeline, PacketPipeline, py::smart_holder>(m, "CpuPacketPipeline")
    .def(py::init<>());

  std::vector<std::string> available = { "cpu" };

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  py::class_<OpenGLPacketPipeline, PacketPipeline, py::smart_holder>(m, "OpenGLPacketPipeline")
    .def(py::init([](bool debug) { return new OpenGLPacketPipeline(nullptr, debug); }),
         py::arg("debug") = false);
  available.emplace_back("opengl");
#endif

#ifdef LIBFREENECT2_WITH_VULKAN_SUPPORT
  py::class_<VulkanPacketPipeline, PacketPipeline, py::smart_holder>(m, "VulkanPacketPipeline")
    .def(py::init<int>(), py::arg("device_id") = -1);
  available.emplace_back("vulkan");
#endif

#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  py::class_<OpenCLPacketPipeline, PacketPipeline, py::smart_holder>(m, "OpenCLPacketPipeline")
    .def(py::init<int>(), py::arg("device_id") = -1);
  py::class_<OpenCLKdePacketPipeline, PacketPipeline, py::smart_holder>(m, "OpenCLKdePacketPipeline")
    .def(py::init<int>(), py::arg("device_id") = -1);
  available.emplace_back("opencl");
  available.emplace_back("opencl_kde");
#endif

#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
  py::class_<CudaPacketPipeline, PacketPipeline, py::smart_holder>(m, "CudaPacketPipeline")
    .def(py::init<int>(), py::arg("device_id") = -1);
  py::class_<CudaKdePacketPipeline, PacketPipeline, py::smart_holder>(m, "CudaKdePacketPipeline")
    .def(py::init<int>(), py::arg("device_id") = -1);
  available.emplace_back("cuda");
  available.emplace_back("cuda_kde");
#endif

  m.def("available_pipelines", [available]() { return available; },
        "Names of the PacketPipeline backends actually compiled into this build.");
}

} // namespace pyfreenect2
