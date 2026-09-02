#pragma once

namespace pybind11 { class module_; }

namespace pyfreenect2
{

void bindLogging(pybind11::module_ &m);
void bindSettings(pybind11::module_ &m);
void bindFrame(pybind11::module_ &m);
void bindPipeline(pybind11::module_ &m);
void bindCore(pybind11::module_ &m);
void bindRegistration(pybind11::module_ &m);
void bindAudio(pybind11::module_ &m);

} // namespace pyfreenect2
