# src/turbojpeg/CMakeLists.txt — TurboJPEG RGB processing backend

add_library(freenect2_turbojpeg)

target_sources(freenect2_turbojpeg PRIVATE
  turbo_jpeg_rgb_packet_processor.cpp
)

freenect2_setup_backend_library(freenect2_turbojpeg)

target_link_libraries(freenect2_turbojpeg
  PRIVATE freenect2_support freenect2_logging
)

if(VCPKG_MANIFEST_MODE)
  target_link_libraries(freenect2_turbojpeg
    PRIVATE libjpeg-turbo::turbojpeg spdlog::spdlog
  )
else()
  target_include_directories(freenect2_turbojpeg PRIVATE ${TurboJPEG_INCLUDE_DIRS})
  target_link_libraries(freenect2_turbojpeg
    PRIVATE ${TurboJPEG_LIBRARIES} spdlog::spdlog
  )
endif()
