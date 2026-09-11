# src/logging/CMakeLists.txt — freenect2_logging: zero-dependency shared
# foundation used by core freenect2 and every backend. Owns:
#  - Logger public API + WithPerfLogging (genuine process-wide singleton state
#    — the global spdlog logger instance)
#  - Frame (not stateful, but must live in a real DLL: freenect2_support is
#    STATIC and carries no export decoration, so a Frame compiled there would
#    never reach freenect2.dll's export table, breaking external consumers
#    like Protonect that construct Frame objects directly)
#  - Device reset coordination (process-wide state: which of core freenect2
#    or freenect2_audio performs the Kinect's one-time device-wide USB
#    reset, since only one of them should -- see
#    device_reset_coordination.h)
#
# Always a real SHARED library, regardless of BUILD_SHARED_LIBS: a static
# library would give every consuming DLL its own private copy of the logger,
# breaking setGlobalLogger()/setLogLevel() for anything outside whichever DLL
# called them. Being its own DLL with zero dependencies also avoids the
# circular freenect2<->backend link that would result from routing logging
# (and Frame) through core freenect2 (which itself links every backend).

add_library(freenect2_logging SHARED)

target_sources(freenect2_logging
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/logging.cpp
    ${PROJECT_SOURCE_DIR}/src/logging_spdlog.cpp
    ${PROJECT_SOURCE_DIR}/src/frame.cpp
    ${PROJECT_SOURCE_DIR}/src/device_reset_coordination.cpp
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/logger.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/logger_spdlog.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/logging.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/perf_logging.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/device_reset_coordination.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/frame_listener.hpp
)

# BUILD_INTERFACE/INSTALL_INTERFACE-wrapped (not raw absolute paths) so this target can actually
# be install(EXPORT)-ed - see src/CMakeLists.txt's freenect2 target for the fuller explanation of
# why an unwrapped absolute build-tree path in PUBLIC include dirs blocks that outright.
# include/internal has no INSTALL_INTERFACE counterpart on purpose (not installed, build-only).
target_include_directories(freenect2_logging
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/internal>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(freenect2_logging PUBLIC cxx_std_17)
set_target_properties(freenect2_logging PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(freenect2_logging PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

include(GenerateExportHeader)
generate_export_header(freenect2_logging
  BASE_NAME freenect2_logging
  EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/freenect2_logging/export.h
)

# Installed because the public logger.h / frame_listener.hpp include it.
include(GNUInstallDirs)
install(FILES ${PROJECT_BINARY_DIR}/freenect2_logging/export.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/freenect2_logging
)

# fmt is PUBLIC: the internal logging.h (used by core + every backend, none
# of it installed) formats with fmt. spdlog is PRIVATE: only logging_spdlog.cpp
# needs it, and only a consumer that includes <libfreenect2/logger_spdlog.h>
# pulls spdlog into its own build.
target_link_libraries(freenect2_logging
  PUBLIC fmt::fmt
  PRIVATE spdlog::spdlog
)

install(TARGETS freenect2_logging
  EXPORT freenect2Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
