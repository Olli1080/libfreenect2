# libfreenect2 — Main CMakeLists.txt
# Modernized following REACH cmake/Options.cmake + FindDependencies.cmake pattern

cmake_minimum_required(VERSION 3.16)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# --- Load options before project() for vcpkg feature mapping ---
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Options.cmake")

# --- Version ---
set(PROJECT_VER_MAJOR 0)
set(PROJECT_VER_MINOR 2)
set(PROJECT_VER_PATCH 0)
set(PROJECT_VER "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")
set(PROJECT_APIVER "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}")

# --- Project ---
project(libfreenect2 LANGUAGES C CXX)

# --- Standard CMake modules ---
include(GNUInstallDirs)

# --- Legacy module path for Find*.cmake fallback ---
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

# --- Resource generation ---
include(GenerateResources)

# --- Shared backend-DLL setup (include dirs, visibility, export header, install) ---
include(AddBackendLibrary)

# --- Output directories ---
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# --- Load modular cmake/ files ---
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindDependencies.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")

# --- Build the library (sets LIBFREENECT2_WITH_*_SUPPORT flags) ---
add_subdirectory(src)

# --- Set feature flags for config.h (from flags set during build) ---
set(HAVE_CXX17 yes)

set(HAVE_VideoToolbox "no (Apple only)")
if(LIBFREENECT2_WITH_VT_SUPPORT)
  set(HAVE_VideoToolbox yes)
endif()

set(HAVE_VAAPI disabled)
if(LIBFREENECT2_WITH_VAAPI_SUPPORT)
  set(HAVE_VAAPI yes)
else()
  set(HAVE_VAAPI no)
endif()

set(HAVE_TegraJPEG disabled)
# TegraJPEG removed — dead hardware (Jetson TK1)

set(HAVE_TurboJPEG no)
if(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT)
  set(HAVE_TurboJPEG yes)
endif()

set(HAVE_OpenGL disabled)
if(LIBFREENECT2_WITH_OPENGL_SUPPORT)
  set(HAVE_OpenGL yes)
else()
  set(HAVE_OpenGL no)
endif()

set(HAVE_OpenCL disabled)
if(LIBFREENECT2_WITH_OPENCL_SUPPORT)
  set(HAVE_OpenCL yes)
else()
  set(HAVE_OpenCL no)
endif()

set(HAVE_CUDA disabled)
if(LIBFREENECT2_WITH_CUDA_SUPPORT)
  set(HAVE_CUDA yes)
else()
  set(HAVE_CUDA no)
endif()

set(HAVE_Examples disabled)
if(BUILD_EXAMPLES)
  set(HAVE_Examples yes)
endif()

set(HAVE_OpenNI2 disabled)
if(BUILD_OPENNI2_DRIVER)
  if(LIBFREENECT2_WITH_OPENNI2_SUPPORT)
    set(HAVE_OpenNI2 yes)
  else()
    set(HAVE_OpenNI2 "no (OpenNI2 not found)")
  endif()
endif()

set(HAVE_streamer_recorder disabled)
if(BUILD_STREAMER_RECORDER)
  set(HAVE_streamer_recorder yes)
endif()

set(HAVE_Audio disabled)
if(LIBFREENECT2_WITH_AUDIO_SUPPORT)
  set(HAVE_Audio yes)
else()
  set(HAVE_Audio no)
endif()

set(HAVE_Python disabled)
if(BUILD_PYTHON_BINDINGS)
  set(HAVE_Python yes)
endif()

# --- Config header ---
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/libfreenect2/config.h.in"
  "${PROJECT_BINARY_DIR}/libfreenect2/config.h"
  @ONLY
)

# --- CMake config files for downstream consumers ---
configure_file(
  freenect2.cmake.in
  "${PROJECT_BINARY_DIR}/freenect2Config.cmake"
  @ONLY
)
configure_file(
  freenect2Version.cmake.in
  "${PROJECT_BINARY_DIR}/freenect2ConfigVersion.cmake"
  @ONLY
)
configure_file(
  freenect2.pc.in
  "${PROJECT_BINARY_DIR}/freenect2.pc"
  @ONLY
)

# --- Windows default install prefix ---
if(WIN32)
  if(NOT DEFINED CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "installation path" FORCE)
  endif()
endif()

# --- RPATH handling ---
if(NOT DEFINED CMAKE_INSTALL_RPATH AND LibUSB_LIBRARY)
  get_filename_component(LibUSB_LIBDIR ${LibUSB_LIBRARY} DIRECTORY)
  if(NOT ${LibUSB_LIBDIR} MATCHES "^/usr/lib")
    set(CMAKE_INSTALL_RPATH ${LibUSB_LIBDIR} CACHE STRING "Set RPATH for private libusb")
  endif()
endif()
if(DEFINED CMAKE_INSTALL_RPATH)
  message(STATUS "RPATH set to ${CMAKE_INSTALL_RPATH}")
endif()

# --- OpenCL + CUDA RPATH conflict workaround ---
if(LIBFREENECT2_WITH_OPENCL_SUPPORT AND UNIX AND NOT APPLE)
  file(GLOB CUDA_ld_so_conf /etc/ld.so.conf.d/cuda*.conf)
  if(CUDA_ld_so_conf)
    message(WARNING "Your CUDA installation overrides OpenCL system library path.")
    if(OpenCL_LIBRARY)
      get_filename_component(OpenCL_LIB_PATH ${OpenCL_LIBRARY} DIRECTORY)
      set(CMAKE_INSTALL_RPATH ${OpenCL_LIB_PATH} CACHE STRING "Set RPATH for system OpenCL" FORCE)
    endif()
  endif()
endif()

# --- Installation ---
install(TARGETS freenect2
  EXPORT freenect2Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/libfreenect2/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libfreenect2
  PATTERN "*.in" EXCLUDE
)
install(DIRECTORY ${PROJECT_BINARY_DIR}/libfreenect2/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libfreenect2
)
install(FILES
  "${PROJECT_BINARY_DIR}/freenect2Config.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/freenect2
)
install(FILES
  "${PROJECT_BINARY_DIR}/freenect2ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/freenect2
)
install(FILES
  "${PROJECT_BINARY_DIR}/freenect2.pc"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
#TODO::fix later
#install(EXPORT freenect2Targets
#  FILE freenect2Targets.cmake
#  NAMESPACE freenect2::
#  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/freenect2
#)

# --- Documentation ---
add_subdirectory(doc)

# --- Examples ---
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# --- OpenNI2 Driver ---
if(BUILD_OPENNI2_DRIVER AND LIBFREENECT2_WITH_OPENNI2_SUPPORT)
  add_subdirectory(src/openni2)
  install(TARGETS freenect2-openni2
    LIBRARY DESTINATION lib/OpenNI2/Drivers
    RUNTIME DESTINATION bin
  )
  add_custom_target(install-openni2
    DEPENDS freenect2-openni2
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${CMAKE_INSTALL_PREFIX}/lib/OpenNI2/Drivers/"
      "${OpenNI2_LIBRARY_DIR}/OpenNI2/Drivers/"
    VERBATIM
  )
endif()

# --- Streamer Recorder ---
if(BUILD_STREAMER_RECORDER)
  add_subdirectory(tools/streamer_recorder)
endif()

# --- Python Bindings ---
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(src/python)
endif()

# --- Feature Summary ---
message(STATUS "Feature summary:")
message(STATUS "  C++17:         ${HAVE_CXX17}")
message(STATUS "  TurboJPEG:     ${HAVE_TurboJPEG}")
message(STATUS "  VideoToolbox:  ${HAVE_VideoToolbox}")
message(STATUS "  OpenGL:        ${HAVE_OpenGL}")
message(STATUS "  OpenCL:        ${HAVE_OpenCL}")
message(STATUS "  CUDA:          ${HAVE_CUDA}")
message(STATUS "  VAAPI:         ${HAVE_VAAPI}")
message(STATUS "  OpenNI2:       ${HAVE_OpenNI2}")
message(STATUS "  Examples:      ${HAVE_Examples}")
message(STATUS "  Streamer:      ${HAVE_streamer_recorder}")
message(STATUS "  Audio:         ${HAVE_Audio}")
message(STATUS "  Python:        ${HAVE_Python}")
