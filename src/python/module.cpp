// src/python/module.cpp — pyfreenect2 module entry point.
//
// Binding surface is deliberately split one topic per translation unit
// (bind_core/bind_frame/bind_pipeline/bind_registration/bind_settings/
// bind_logging/bind_audio) so each stays reviewable on its own, mirroring
// how the C++ public API itself is split across headers.

#include "bindings.h"

#include <libfreenect2/config.h>

#include <pybind11/pybind11.h>

PYBIND11_MODULE(pyfreenect2, m)
{
  m.doc() =
    "Python bindings for libfreenect2 (Kinect v2 driver): device "
    "enumeration/control, video+depth capture via SyncMultiFrameListener, "
    "depth/color registration, and (when built with audio support) "
    "microphone-array capture and direction-of-arrival estimation.\n\n"
    "v1 scope: only the polling (SyncMultiFrameListener/SyncAudioListener) "
    "consumption model is exposed. Subclassing FrameListener/"
    "AudioFrameListener from Python is not yet supported.";

  pyfreenect2::bindLogging(m);
  pyfreenect2::bindSettings(m);
  pyfreenect2::bindFrame(m);
  pyfreenect2::bindPipeline(m);
  pyfreenect2::bindCore(m);
  pyfreenect2::bindRegistration(m);

#ifdef LIBFREENECT2_WITH_AUDIO_SUPPORT
  pyfreenect2::bindAudio(m);
#endif
}
