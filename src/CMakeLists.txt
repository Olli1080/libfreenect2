# src/CMakeLists.txt — libfreenect2 core library
# Per-module CMakeLists.txt following REACH pattern:
#   target_include_directories() with $<BUILD_INTERFACE:...>
#   target_sources() with PUBLIC FILE_SET HEADERS
#   target_compile_features() for C++ standard
#   target_link_libraries() with PUBLIC/PRIVATE visibility

add_library(freenect2)

# --- Include Directories ---
target_include_directories(freenect2
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/internal>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

# --- C++ Standard ---
target_compile_features(freenect2 PUBLIC cxx_std_17)
set_target_properties(freenect2 PROPERTIES CXX_STANDARD_REQUIRED ON)

# --- Target Properties ---
set_target_properties(freenect2 PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  VERSION ${PROJECT_VER}
  SOVERSION ${PROJECT_APIVER}
  OUTPUT_NAME freenect2
)

# --- Generate Export Header ---
include(GenerateExportHeader)
generate_export_header(freenect2
  BASE_NAME libfreenect2
  EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/libfreenect2/export.h
)

# --- Core Sources (always compiled) ---
target_sources(freenect2
  PRIVATE
    transfer_pool.cpp
    event_loop.cpp
    usb_control.cpp
    frame_listener_impl.cpp
    packet_pipeline.cpp
    rgb_packet_stream_parser.cpp
    depth_packet_stream_parser.cpp
    cpu_depth_packet_processor.cpp
    command_transaction.cpp
    registration.cpp
    libfreenect2.cpp
  PUBLIC FILE_SET HEADERS
  BASE_DIRS ${PROJECT_SOURCE_DIR}/include
  FILES
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/logger.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/frame_listener.hpp
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/frame_listener_impl.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/libfreenect2.hpp
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/color_settings.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/led_settings.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/packet_pipeline.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/registration.h
)

# --- Internal Headers ---
target_sources(freenect2
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/protocol/command.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/protocol/command_transaction.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/protocol/response.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/protocol/usb_control.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/usb/event_loop.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/usb/transfer_pool.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/depth_packet_stream_parser.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/rgb_packet_stream_parser.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/data_callback.h
)

# --- Required Dependencies ---
target_link_libraries(freenect2
  PUBLIC
    libusb::libusb-1.0
    spdlog::spdlog
)

# --- Logging Library ---
# The process-wide logger singleton (Logger public API + WithPerfLogging).
# A real SHARED library with zero dependencies so every backend can link it
# directly without creating a cycle back through core freenect2 (which links
# every backend). PUBLIC so downstream consumers (Protonect, etc.) that only
# link freenect2 still transitively get it.
add_subdirectory(logging)
target_link_libraries(freenect2 PUBLIC freenect2_logging)

# --- Support Library ---
# Foundational internals (Allocator, DepthPacketProcessor/RgbPacketProcessor
# bases, resource loading) shared by core and every backend. Must be added
# before the backend subdirectories below, which also link against it.
add_subdirectory(support)
target_link_libraries(freenect2 PRIVATE freenect2_support)

# --- Backend Subdirectories ---
# Each backend is a self-contained subdirectory following REACH pattern.
# Dependencies are discovered in cmake/FindDependencies.cmake before this point.

# --- TurboJPEG (always needed unless VideoToolbox on macOS) ---
if(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT)
  add_subdirectory(turbojpeg)
  target_link_libraries(freenect2 PRIVATE freenect2_turbojpeg)
endif()

# --- macOS VideoToolbox (automatic) ---
if(APPLE AND LIBFREENECT2_WITH_VT_SUPPORT)
  add_subdirectory(vt)
  target_link_libraries(freenect2 PRIVATE freenect2_vt)
endif()

# --- OpenGL Backend ---
if(LIBFREENECT2_WITH_OPENGL_SUPPORT)
  add_subdirectory(opengl)
  target_link_libraries(freenect2 PRIVATE freenect2_opengl)
endif()

# --- OpenCL Backend ---
if(LIBFREENECT2_WITH_OPENCL_SUPPORT)
  add_subdirectory(opencl)
  target_link_libraries(freenect2 PRIVATE freenect2_opencl)
endif()

# --- CUDA Backend ---
if(ENABLE_CUDA)
  set(CMAKE_CUDA_RUNTIME_LIBRARY Static)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    enable_language(CUDA OPTIONAL)
    if(CUDAToolkit_NVCC_EXECUTABLE)
      add_subdirectory(cuda)
      target_link_libraries(freenect2 PRIVATE freenect2_cuda CUDA::cudart)
      set(LIBFREENECT2_WITH_CUDA_SUPPORT 1 CACHE INTERNAL "CUDA supported")
    endif()
  endif()
endif()

# --- VAAPI Backend (deprecated) ---
if(LIBFREENECT2_WITH_VAAPI_SUPPORT)
  add_subdirectory(vaapi)
  target_link_libraries(freenect2 PRIVATE freenect2_vaapi)
endif()

# --- Audio Backend (Kinect v2 mic array capture) ---
# Deliberately NOT linked into core freenect2 (target_link_libraries below is
# absent on purpose) — freenect2 stays video/depth-only per its documented
# scope. Applications that want audio link freenect2_audio directly.
if(ENABLE_AUDIO)
  add_subdirectory(audio)
  set(LIBFREENECT2_WITH_AUDIO_SUPPORT 1 CACHE INTERNAL "Audio supported")
endif()

# --- Static Build Suffix (MSVC) ---
if(MSVC AND NOT BUILD_SHARED_LIBS)
  set_target_properties(freenect2 PROPERTIES SUFFIX "static.lib")
endif()

# --- Output Directories ---
set_target_properties(freenect2 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)
