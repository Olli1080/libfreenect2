# src/python/CMakeLists.txt — pyfreenect2 Python bindings (pybind11)
#
# Opt-in (BUILD_PYTHON_BINDINGS, default OFF). Links against the umbrella
# freenect2 target for video/depth/registration/pipelines -- same pattern as
# freenect2-openni2 (src/openni2/CMakeLists.txt): PRIVATE freenect2 is
# sufficient, since the individual backend libs are already linked PRIVATE
# into freenect2 itself. Additionally links freenect2_audio when audio
# support is compiled in, mirroring how any other external consumer links it
# (see src/audio/CMakeLists.txt) -- freenect2_audio is deliberately not
# linked into core freenect2.

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(pyfreenect2
  module.cpp
  bind_core.cpp
  bind_frame.cpp
  bind_pipeline.cpp
  bind_registration.cpp
  bind_settings.cpp
  bind_logging.cpp
)

target_include_directories(pyfreenect2 PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_BINARY_DIR}
)

target_compile_features(pyfreenect2 PRIVATE cxx_std_17)

target_link_libraries(pyfreenect2 PRIVATE freenect2)

if(LIBFREENECT2_WITH_AUDIO_SUPPORT)
  target_sources(pyfreenect2 PRIVATE bind_audio.cpp)
  target_link_libraries(pyfreenect2 PRIVATE freenect2_audio)
endif()

set_target_properties(pyfreenect2 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)
