// src/python/bind_audio.cpp — AudioFrame, AudioCapture, SyncAudioListener,
// DoaEstimate, DoaEstimator.
//
// Only compiled/linked when LIBFREENECT2_WITH_AUDIO_SUPPORT (see
// src/python/CMakeLists.txt) -- module.cpp only calls bindAudio() under the
// same guard.

#include "bindings.h"
#include "sync_audio_listener.h"

#include <libfreenect2/audio_capture.hpp>
#include <libfreenect2/audio_doa.hpp>
#include <libfreenect2/audio_ring_buffer.hpp>
#include <libfreenect2/libfreenect2.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace libfreenect2;

namespace pyfreenect2
{

namespace
{

bool audioStart(AudioCapture &self, SyncAudioListener &listener)
{
  py::gil_scoped_release release;
  return self.start(&listener);
}

void audioStop(AudioCapture &self)
{
  py::gil_scoped_release release;
  self.stop();
}

py::buffer_info audioFrameBuffer(AudioFrame &frame)
{
  // samples is interleaved ch0..ch3 -> (frame_count, channels) float32.
  return py::buffer_info(
    frame.samples.data(),
    sizeof(float),
    py::format_descriptor<float>::format(),
    2,
    { static_cast<py::ssize_t>(frame.frame_count), static_cast<py::ssize_t>(frame.channels) },
    { static_cast<py::ssize_t>(frame.channels * sizeof(float)), static_cast<py::ssize_t>(sizeof(float)) }
  );
}

} // namespace

void bindAudio(py::module_ &m)
{
  m.def("now_microseconds", &nowMicroseconds,
        "Monotonic clock, microseconds -- same clock as AudioFrame.timestamp_us.");

  py::class_<AudioFrame>(m, "AudioFrame", py::buffer_protocol(),
      "One buffer's worth of captured mic-array audio. Supports the buffer "
      "protocol: numpy.asarray(frame) gives a (frame_count, channels) "
      "float32 array.")
    .def(py::init<>())
    .def_readonly("frame_count", &AudioFrame::frame_count)
    .def_readonly("channels", &AudioFrame::channels)
    .def_readonly("sample_rate", &AudioFrame::sample_rate)
    .def_readonly("timestamp_us", &AudioFrame::timestamp_us)
    .def_buffer(&audioFrameBuffer);

  py::class_<AudioFrameListener>(m, "AudioFrameListener",
      "Base callback interface. Not subclassable from Python in v1 -- use "
      "SyncAudioListener instead.");

  py::class_<SyncAudioListener, AudioFrameListener>(m, "SyncAudioListener")
    .def(py::init<>())
    .def("get_latest_frame", &SyncAudioListener::getLatestFrame,
         "Returns the most recently captured AudioFrame, or None if no new "
         "frame has arrived since the last call. Overwrite-on-arrival, not "
         "a queue -- frames are dropped, not buffered, if not polled often "
         "enough.");

  py::class_<AudioCapture>(m, "AudioCapture")
    .def(py::init<>(),
         "Opens an independent USB session. On Windows/UsbDk this fails if "
         "a Freenect2Device for the same physical Kinect is already open -- "
         "use the other constructor for simultaneous audio+video capture.")
    .def(py::init<Freenect2Device *>(), py::arg("device"), py::keep_alive<1, 2>(),
         "Shares an already-open Freenect2Device's USB session. `device` "
         "must outlive this AudioCapture.")
    .def("is_available", &AudioCapture::isAvailable)
    .def("start", &audioStart, py::arg("listener"), py::keep_alive<1, 2>())
    .def("stop", &audioStop);

  py::class_<DoaEstimate>(m, "DoaEstimate")
    .def_readonly("azimuth_degrees", &DoaEstimate::azimuth_degrees)
    .def_readonly("confidence", &DoaEstimate::confidence)
    .def_readonly("valid", &DoaEstimate::valid);

  py::class_<DoaEstimator>(m, "DoaEstimator")
    .def(py::init<>())
    .def("add_samples", &DoaEstimator::addSamples, py::arg("frame"),
         "Call from the same thread that polls SyncAudioListener.")
    .def("estimate_gcc_phat", &DoaEstimator::estimateGccPhat)
    .def("estimate_beamforming", &DoaEstimator::estimateBeamforming);

  py::class_<AudioRingBuffer>(m, "AudioRingBuffer",
      "Thread-safe rolling window of recently-captured AudioFrames, "
      "queryable by timestamp -- for correlating audio with a video Frame's "
      "arrival time (tag arrival with now_microseconds(), pass it to query()).")
    .def(py::init<uint64_t>(), py::arg("retention_us") = 5000000,
         "retention_us: how much audio history to keep before older frames "
         "are dropped. Must be at least as large as the largest window you "
         "intend to query() with.")
    .def("push", &AudioRingBuffer::push, py::arg("frame"),
         "Copies frame internally -- call from an AudioFrameListener/poll loop.")
    .def("query", &AudioRingBuffer::query, py::arg("center_us"), py::arg("window_us"),
         "Returns copies of all retained frames whose timestamp_us falls "
         "within [center_us - window_us/2, center_us + window_us/2], oldest first.");
}

} // namespace pyfreenect2
