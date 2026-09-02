// src/python/bind_logging.cpp — log-level control.
//
// Only setLogLevel()/getLogLevel() are bound. spdlog::logger/sink objects
// themselves are not exposed (out of scope for v1) -- Python code can
// silence or raise the library's own logger via LogLevel, but cannot
// install a custom Python-side sink.

#include "bindings.h"

#include <libfreenect2/logger.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

void bindLogging(py::module_ &m)
{
  py::enum_<spdlog::level::level_enum>(m, "LogLevel")
    .value("Trace", spdlog::level::trace)
    .value("Debug", spdlog::level::debug)
    .value("Info", spdlog::level::info)
    .value("Warn", spdlog::level::warn)
    .value("Error", spdlog::level::err)
    .value("Critical", spdlog::level::critical)
    .value("Off", spdlog::level::off);

  m.def("get_log_level", &getLogLevel);
  m.def("set_log_level", &setLogLevel, py::arg("level"));
}

} // namespace pyfreenect2
