# src/vt/CMakeLists.txt — macOS VideoToolbox RGB processing backend

add_library(freenect2_vt)

target_sources(freenect2_vt PRIVATE vt_rgb_packet_processor.cpp)

freenect2_setup_backend_library(freenect2_vt)

target_link_libraries(freenect2_vt
  PRIVATE ${VIDEOTOOLBOX_LIBRARY} ${COREFOUNDATION_LIBRARY} ${COREMEDIA_LIBRARY} ${COREVIDEO_LIBRARY}
  PRIVATE freenect2_support freenect2_logging
)
