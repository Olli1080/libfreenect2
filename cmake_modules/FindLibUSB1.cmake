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
