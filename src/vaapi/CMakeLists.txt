# src/vaapi/CMakeLists.txt — Intel VA-API hardware JPEG decode backend (Linux, off by default)

add_library(freenect2_vaapi)

target_sources(freenect2_vaapi PRIVATE
  vaapi_rgb_packet_processor.cpp
)

freenect2_setup_backend_library(freenect2_vaapi)
target_include_directories(freenect2_vaapi PRIVATE ${VAAPI_INCLUDE_DIRS})

target_link_libraries(freenect2_vaapi
  PRIVATE ${VAAPI_LIBRARIES} ${JPEG_LIBRARY} spdlog::spdlog freenect2_support freenect2_logging
)
