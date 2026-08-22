# src/logging/CMakeLists.txt — freenect2_logging: zero-dependency shared
# foundation used by core freenect2 and every backend. Owns:
#  - Logger public API + WithPerfLogging (genuine process-wide singleton state
#    — the global spdlog logger instance)
#  - Frame (not stateful, but must live in a real DLL: freenect2_support is
#    STATIC and carries no export decoration, so a Frame compiled there would
#    never reach freenect2.dll's export table, breaking external consumers
#    like Protonect that construct Frame objects directly)
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
    ${PROJECT_SOURCE_DIR}/src/frame.cpp
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/logger.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/logging.h
    ${PROJECT_SOURCE_DIR}/include/internal/libfreenect2/perf_logging.h
    ${PROJECT_SOURCE_DIR}/include/libfreenect2/frame_listener.hpp
)

target_include_directories(freenect2_logging
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/internal
    ${PROJECT_BINARY_DIR}
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

target_link_libraries(freenect2_logging
  PUBLIC spdlog::spdlog
)

install(TARGETS freenect2_logging
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
