// src/python/bind_logging.cpp — log-level and sink control.
//
// LogLevel + set_log_level()/get_log_level() control how much the library
// logs (LogLevel.None silences it). set_log_sink() routes every line to a
// Python callable(level, message); passing None restores the stderr sink.

#include "bindings.h"

#include <libfreenect2/logger.h>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

void bindLogging(py::module_ &m)
{
  py::enum_<LogLevel>(m, "LogLevel")
    .value("None_", LogLevel::None)   // `None` is a Python keyword
    .value("Error", LogLevel::Error)
    .value("Warning", LogLevel::Warning)
    .value("Info", LogLevel::Info)
    .value("Debug", LogLevel::Debug);

  m.def("get_default_level", &getDefaultLevel);
  m.def("get_log_level", &getLogLevel);
  m.def("set_log_level", &setLogLevel, py::arg("level"));
  m.def("set_log_sink",
        [](py::object sink) {
          if (sink.is_none())
          {
            setLogSink(nullptr);
            return;
          }
          auto fn = sink.cast<std::function<void(LogLevel, std::string)>>();
          setLogSink([fn](LogLevel level, std::string_view message) {
            py::gil_scoped_acquire gil;
            fn(level, std::string(message));
          });
        },
        py::arg("sink"),
        "Route library log output to callable(level, message), or None for "
        "the default stderr sink.");
}

} // namespace pyfreenect2
