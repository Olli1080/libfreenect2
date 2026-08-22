# src/openni2/CMakeLists.txt — OpenNI2 driver plugin

add_library(freenect2-openni2)

target_sources(freenect2-openni2 PRIVATE
  ColorStream.cpp
  DepthStream.cpp
  DeviceDriver.cpp
  IrStream.cpp
  Registration.cpp
  Utility.cpp
  VideoStream.cpp
)

target_include_directories(freenect2-openni2
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/internal
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${OpenNI2_INCLUDE_DIRS}
)

target_compile_features(freenect2-openni2 PRIVATE cxx_std_17)

target_link_libraries(freenect2-openni2
  PRIVATE
    freenect2
    ${OpenNI2_LIBRARIES}
)

set_target_properties(freenect2-openni2 PROPERTIES
  SOVERSION 0
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)

if(NOT ${CMAKE_INSTALL_PREFIX} MATCHES "^/usr")
  set_target_properties(freenect2-openni2 PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
  )
endif()
