# src/support/CMakeLists.txt — freenect2_support: foundational internals shared
# by core freenect2 and every backend (opencl, opengl, cuda, turbojpeg, vaapi, vt).
#
# Always STATIC regardless of BUILD_SHARED_LIBS: every consumer gets its own
# compiled copy, which is safe here because nothing in this target holds
# process-wide singleton state (the logger lives in freenect2_logging instead).
# Frame also lives in freenect2_logging, not here — a STATIC library carries no
# export decoration, so a Frame compiled here would never reach freenect2.dll's
# export table, breaking external consumers (Protonect etc.) that construct
# Frame objects directly. freenect2_logging is a real DLL, so it can genuinely
# export Frame once and have every consumer (core, every backend, and external
# apps via core's PUBLIC link) share that one exported implementation.
#
# Source/header files stay in their existing src/ and include/internal/libfreenect2/
# locations (unchanged include paths for the rest of the tree) — this target just
# takes over compiling/owning them instead of core `freenect2`.

add_library(freenect2_support STATIC)

target_sources(freenect2_support
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/allocator.cpp
    ${PROJECT_SOURCE_DIR}/src/depth_packet_processor.cpp
    ${PROJECT_SOURCE_DIR}/src/rgb_packet_processor.cpp
    ${PROJECT_SOURCE_DIR}/src/resource.cpp
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/allocator.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/packet_processor.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/async_packet_processor.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/depth_packet_processor.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/rgb_packet_processor.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/resource.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/threading.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/depth_processor_config.h
)

target_include_directories(freenect2_support
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/internal
    ${PROJECT_BINARY_DIR}
)

target_compile_features(freenect2_support PUBLIC cxx_std_17)
set_target_properties(freenect2_support PROPERTIES CXX_STANDARD_REQUIRED ON)

target_link_libraries(freenect2_support
  PUBLIC
  PRIVATE freenect2_logging
)

# --- Resource Embedding ---
# Only core's calibration .yml files — OpenCL kernels and OpenGL shaders are
# embedded by their own backends instead (src/opencl, src/opengl), so a given
# backend DLL doesn't end up embedding another backend's unrelated resource
# source text. Since freenect2_support is STATIC, every consumer (core and
# every backend) gets its own copy of loadResource()/this table — harmless
# duplication of a few small read-only YAML placeholders.
generate_resources_target(RESOURCES_INC_FILE core
  src/data/kinect2_lr_real.yml
  src/data/kinect2_lr_imag.yml
  src/data/kinect2_bg_real.yml
  src/data/kinect2_bg_imag.yml
  src/data/kinect2_depth_reg_real.yml
  src/data/kinect2_depth_reg_imag.yml
)
target_sources(freenect2_support PRIVATE ${RESOURCES_INC_FILE})
target_compile_definitions(freenect2_support PRIVATE RESOURCES_INC RESOURCES_INC_PATH="resources_core.inc.h")
