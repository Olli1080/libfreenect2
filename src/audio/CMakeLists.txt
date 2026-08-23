# src/audio/CMakeLists.txt — Kinect v2 microphone array capture backend
#
# Core freenect2 does not link this back (see src/CMakeLists.txt — freenect2
# stays video/depth-only per its documented scope), but this DOES link core
# freenect2 (one-way: audio -> core), specifically so AudioCapture can reuse
# an already-open Freenect2Device's USB session via Freenect2Device::
# getUsbContext()/getUsbDeviceHandle() -- required for simultaneous
# audio+video capture, since on Windows with the UsbDk backend only one
# libusb_open() session per physical device is permitted at all, so
# freenect2_audio cannot always open its own independent one (see
# audio_capture.cpp for the full story, including the AudioCapture()
# no-arg constructor that still works standalone with its own session).
#
# Built directly on libusb (already an unconditional project-wide
# dependency, no new vcpkg package needed) rather than an OS audio API: the
# mic array's isochronous streaming interface is read the same way core
# libfreenect2 reads video/depth. This isn't a style choice — a
# WASAPI/RtAudio-based capture session cannot survive the device-wide USB
# reset libfreenect2 performs when opening the video device, so a plain
# OS-audio-API backend cannot ever support simultaneous audio+video capture.

add_library(freenect2_audio)

target_sources(freenect2_audio PRIVATE
  audio_capture.cpp
  audio_ring_buffer.cpp
)

freenect2_setup_backend_library(freenect2_audio)

# Unlike the other backends, freenect2_audio is meant to be linked directly
# by external consumers (nothing links it into core freenect2 — see above),
# so its public header (audio_capture.hpp) and generated export header need
# to propagate to them. freenect2_setup_backend_library() sets include dirs
# PRIVATE, which is correct for the other 6 backends (only core consumes
# them, and core already has its own include paths) — add the two PUBLIC
# paths this one needs on top of that instead of changing the shared
# function's default for everyone.
target_include_directories(freenect2_audio PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_BINARY_DIR}
)

target_link_libraries(freenect2_audio
  PRIVATE freenect2_logging freenect2 libusb::libusb-1.0
)
