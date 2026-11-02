# cmake_modules/FindLibUSB1.cmake — constructs libusb::libusb-1.0.
#
# vcpkg's libusb port ships only a pkg-config file (libusb-1.0.pc), no CMake config - so this
# target has to be hand-built from pkg-config, same reasoning as REACH's own miniaudio/autodiff
# IMPORTED targets for other pkg-config/config-less vcpkg ports.
#
# Factored into its own module (idempotent - safe to include() more than once) rather than
# duplicated inline, because it is needed in TWO places: this project's own build
# (cmake/FindDependencies.cmake) AND freenect2Config.cmake (core freenect2 links libusb PUBLIC,
# so an external consumer's find_package(freenect2 CONFIG) needs this target to exist too) - see
# freenect2.cmake.in's own include() of this file, and the root CMakeLists' install(FILES ...)
# that ships this module alongside the installed config.
if(NOT TARGET libusb::libusb-1.0)
  find_package(PkgConfig QUIET)
  pkg_check_modules(LIBUSB_PC REQUIRED libusb-1.0)

  # pkg-config names one directory, the one of the build type it was configured for. vcpkg keeps the release
  # library in <prefix>/lib and the debug one in <prefix>/debug/lib, so take both: a Release program must not
  # link (and get an rpath to) the debug directory.
  set(LIBUSB_RELEASE_DIRS "")
  set(LIBUSB_DEBUG_DIRS "")
  foreach(LIBDIR IN LISTS LIBUSB_PC_LIBRARY_DIRS)
    # the .pc file of vcpkg builds the directory as <pcfiledir>/../../lib, resolve the ".." first
    get_filename_component(LIBDIR "${LIBDIR}" ABSOLUTE)
    if(LIBDIR MATCHES "^(.*)/debug/lib$")
      list(APPEND LIBUSB_DEBUG_DIRS "${LIBDIR}")
      list(APPEND LIBUSB_RELEASE_DIRS "${CMAKE_MATCH_1}/lib")
    else()
      list(APPEND LIBUSB_RELEASE_DIRS "${LIBDIR}")
      list(APPEND LIBUSB_DEBUG_DIRS "${LIBDIR}/../debug/lib")
    endif()
  endforeach()

  # libusb-1.0.lib (Windows), libusb-1.0.so or libusb-1.0.a (Linux): find_library knows the platform's affixes.
  # A result cached by an earlier configure may be the wrong one, search again.
  unset(LIBUSB_RELEASE_PATH CACHE)
  unset(LIBUSB_DEBUG_PATH CACHE)
  find_library(LIBUSB_RELEASE_PATH NAMES usb-1.0 libusb-1.0 HINTS ${LIBUSB_RELEASE_DIRS} NO_DEFAULT_PATH)
  find_library(LIBUSB_DEBUG_PATH NAMES usb-1.0 libusb-1.0 HINTS ${LIBUSB_DEBUG_DIRS} NO_DEFAULT_PATH)

  add_library(libusb::libusb-1.0 UNKNOWN IMPORTED)
  set_target_properties(libusb::libusb-1.0 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${LIBUSB_PC_INCLUDE_DIRS}"
  )
  if(LIBUSB_RELEASE_PATH)
    set_target_properties(libusb::libusb-1.0 PROPERTIES IMPORTED_LOCATION "${LIBUSB_RELEASE_PATH}")
  endif()
  if(LIBUSB_DEBUG_PATH)
    set_target_properties(libusb::libusb-1.0 PROPERTIES IMPORTED_LOCATION_DEBUG "${LIBUSB_DEBUG_PATH}")
    if(NOT LIBUSB_RELEASE_PATH)
      set_target_properties(libusb::libusb-1.0 PROPERTIES IMPORTED_LOCATION "${LIBUSB_DEBUG_PATH}")
    endif()
  endif()
endif()
