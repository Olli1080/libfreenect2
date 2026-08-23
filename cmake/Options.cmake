# libfreenect2 Build Options
# Modeled after REACH cmake/Options.cmake pattern

# --- Core Build Options ---
option(BUILD_SHARED_LIBS "Build shared (ON) or static (OFF) libraries" ON)
option(BUILD_EXAMPLES "Build examples (Protonect)" ON)
option(BUILD_OPENNI2_DRIVER "Build OpenNI2 driver plugin" ON)
option(BUILD_STREAMER_RECORDER "Build network streaming tool" OFF)
option(BUILD_PCL_EXAMPLE "Build PCL point cloud visualization example" OFF)
option(BUILD_ONNX_EXAMPLES "Build ONNX Runtime object-detection/pose examples" OFF)
option(ENABLE_PROFILING "Collect profiling stats (memory consuming)" OFF)

# --- Backend Options ---
option(ENABLE_OPENCL "Enable OpenCL depth processing" ON)
option(ENABLE_CUDA "Enable CUDA depth processing" ON)
option(ENABLE_OPENGL "Enable OpenGL depth processing + viewer" ON)
option(ENABLE_VAAPI "Enable Intel VA-API JPEG decode (Linux, deprecated)" OFF)
option(ENABLE_AUDIO "Enable Kinect v2 microphone array capture" ON)
option(ENABLE_VULKAN "Enable Vulkan depth processing" ON)

# --- vcpkg Feature Mapping ---
if(ENABLE_OPENCL)
  list(APPEND VCPKG_MANIFEST_FEATURES "opencl")
endif()

if(ENABLE_OPENGL)
  list(APPEND VCPKG_MANIFEST_FEATURES "opengl")
endif()

if(BUILD_OPENNI2_DRIVER)
  list(APPEND VCPKG_MANIFEST_FEATURES "openni2")
endif()

if(BUILD_STREAMER_RECORDER)
  list(APPEND VCPKG_MANIFEST_FEATURES "opencv")
endif()

if(ENABLE_AUDIO)
  list(APPEND VCPKG_MANIFEST_FEATURES "audio")
endif()

if(ENABLE_VULKAN)
  list(APPEND VCPKG_MANIFEST_FEATURES "vulkan")
endif()

if(BUILD_PCL_EXAMPLE)
  list(APPEND VCPKG_MANIFEST_FEATURES "pcl")
endif()

if(BUILD_ONNX_EXAMPLES)
  list(APPEND VCPKG_MANIFEST_FEATURES "onnx")
endif()

# --- Compiler Options ---
if(MSVC)
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/Zc:__cplusplus>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/std:c++17>")

  # Suppress warnings: size conversion, zero-length array in libusb.h, secure CRT
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/wd4267>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/wd4244>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/wd4200>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/wd4305>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/wd4146>")
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
  add_compile_definitions(_WIN32_WINNT=0x0A00)
else()
  add_compile_options("$<$<CXX_COMPILER_ID:GNU>:-Wall>")
  add_compile_options("$<$<CXX_COMPILER_ID:Clang>:-Wall>")
endif()

# --- Global Settings ---
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# --- Debug Postfix (Windows) ---
if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Debug library postfix")
  endif()
endif()

# --- Default Build Type (single-config generators only) ---
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()

# --- Profiling ---
if(ENABLE_PROFILING)
  add_compile_definitions(LIBFREENECT2_WITH_PROFILING=1)
endif()

# --- VAAPI Deprecation Warning ---
if(ENABLE_VAAPI)
  message(DEPRECATION "ENABLE_VAAPI is deprecated. The VAAPI backend uses old VA 0.x API. Use TurboJPEG instead.")
endif()

# --- Policies ---
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()
