# Unified Dependency Discovery for libfreenect2
# Modeled after REACH cmake/FindDependencies.cmake pattern
#
# Supports two modes:
#   1. vcpkg manifest mode — uses find_package(... CONFIG)
#   2. Legacy/manual mode — uses cmake_modules/Find*.cmake scripts

# --- Always Required: libusb ---
# vcpkg's libusb provides only pkg-config, not a CMake config file
# Ensure libusb::libusb-1.0 target exists for downstream CMake usage
if(NOT TARGET libusb::libusb-1.0)
  find_package(PkgConfig QUIET)
  pkg_check_modules(LIBUSB_PC REQUIRED libusb-1.0)

  # Construct full path: libdir/libusb-1.0.lib
  foreach(LIBDIR IN LISTS LIBUSB_PC_LIBRARY_DIRS)
    if(EXISTS "${LIBDIR}/libusb-1.0.lib")
      set(LIBUSB_FULL_PATH "${LIBDIR}/libusb-1.0.lib")
      break()
    elseif(EXISTS "${LIBDIR}/libusb-1.0.a")
      set(LIBUSB_FULL_PATH "${LIBDIR}/libusb-1.0.a")
      break()
    endif()
  endforeach()

  add_library(libusb::libusb-1.0 UNKNOWN IMPORTED)
  set_target_properties(libusb::libusb-1.0 PROPERTIES
    IMPORTED_LOCATION "${LIBUSB_FULL_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_PC_INCLUDE_DIRS}"
  )
endif()

# --- Always Required: TurboJPEG ---
if(VCPKG_MANIFEST_MODE)
  find_package(libjpeg-turbo CONFIG REQUIRED)
  set(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT 1 CACHE INTERNAL "TURBOJPEG supported")
else()
  find_package(TurboJPEG REQUIRED)
  if(TurboJPEG_FOUND)
    set(LIBFREENECT2_WITH_TURBOJPEG_SUPPORT 1 CACHE INTERNAL "TURBOJPEG supported")
  endif()
endif()

# --- Always Required: fmt (internal log formatting), spdlog (opt-in bridge) ---
if(VCPKG_MANIFEST_MODE)
  find_package(fmt CONFIG REQUIRED)
  find_package(spdlog CONFIG REQUIRED)
else()
  find_package(fmt REQUIRED)
  find_package(spdlog REQUIRED)
endif()

# --- macOS VideoToolbox (automatic on Apple) ---
if(APPLE)
  find_library(VIDEOTOOLBOX_LIBRARY VideoToolbox QUIET)
  if(VIDEOTOOLBOX_LIBRARY)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation QUIET)
    find_library(COREMEDIA_LIBRARY CoreMedia QUIET)
    find_library(COREVIDEO_LIBRARY CoreVideo QUIET)
    set(LIBFREENECT2_WITH_VT_SUPPORT 1 CACHE INTERNAL "VT supported")
  endif()
endif()

# --- OpenGL Backend (optional) ---
if(ENABLE_OPENGL)
  find_package(OpenGL REQUIRED)

  if(VCPKG_MANIFEST_MODE)
    find_package(glfw3 CONFIG REQUIRED)
    set(LIBFREENECT2_WITH_OPENGL_SUPPORT 1 CACHE INTERNAL "OpenGL supported")
  else()
    find_package(GLFW3 REQUIRED)
    if(GLFW3_FOUND)
      set(LIBFREENECT2_WITH_OPENGL_SUPPORT 1 CACHE INTERNAL "OpenGL supported")
    endif()
  endif()
endif()

# --- OpenCL Backend (optional) ---
if(ENABLE_OPENCL)
  if(VCPKG_MANIFEST_MODE)
    find_package(OpenCL CONFIG REQUIRED)
    set(LIBFREENECT2_WITH_OPENCL_SUPPORT 1 CACHE INTERNAL "OPENCL supported")
  else()
    find_package(OpenCL REQUIRED)
    if(OpenCL_FOUND)
      set(LIBFREENECT2_WITH_OPENCL_SUPPORT 1 CACHE INTERNAL "OPENCL supported")

      # OpenCL ICD loader check (Linux)
      if(UNIX AND NOT APPLE)
        include(CheckOpenCLICDLoader)
        if(OpenCL_C_WORKS AND NOT OpenCL_CXX_WORKS)
          set(LIBFREENECT2_OPENCL_ICD_LOADER_IS_OLD 1)
          message(WARNING "Your libOpenCL.so is incompatible with CL/cl.h. Install ocl-icd-opencl-dev to update libOpenCL.so?")
        endif()
      endif()
    endif()
  endif()
endif()

# --- CUDA Backend (optional) ---
# CUDA is enabled via enable_language(CUDA) in src/CMakeLists.txt.
# Flag is set there after language is enabled.

# --- VAAPI Backend (optional, deprecated) ---
if(ENABLE_VAAPI)
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(VAAPI QUIET libva libva-drm)
  endif()
  find_package(JPEG QUIET)
  if(VAAPI_FOUND AND JPEG_FOUND)
    set(LIBFREENECT2_WITH_VAAPI_SUPPORT 1 CACHE INTERNAL "VAAPI supported")
  endif()
endif()

# --- OpenNI2 Driver (optional) ---
if(BUILD_OPENNI2_DRIVER)
  if(VCPKG_MANIFEST_MODE)
    find_package(OpenNI2 CONFIG QUIET)
  endif()
  if(NOT TARGET OpenNI2::OpenNI2)
    find_package(OpenNI2 QUIET)
  endif()
  if(TARGET OpenNI2::OpenNI2 OR OpenNI2_FOUND)
    set(LIBFREENECT2_WITH_OPENNI2_SUPPORT 1CACHE INTERNAL "Openni2 supported")
  endif()
endif()
