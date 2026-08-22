# src/opencl/CMakeLists.txt — OpenCL depth processing backend

add_library(freenect2_opencl)

target_sources(freenect2_opencl PRIVATE
  opencl_depth_packet_processor.cpp
  opencl_kde_depth_packet_processor.cpp
  ${PROJECT_SOURCE_DIR}/src/resource.cpp
)

# --- Resource Embedding ---
# Only this backend's own OpenCL kernel source — see src/support/CMakeLists.txt
# for why resources are scoped per-consumer instead of one shared table.
generate_resources_target(RESOURCES_INC_FILE opencl
  src/opencl/opencl_depth_packet_processor.cl
  src/opencl/opencl_kde_depth_packet_processor.cl
)
target_sources(freenect2_opencl PRIVATE ${RESOURCES_INC_FILE})
target_compile_definitions(freenect2_opencl PRIVATE RESOURCES_INC)

freenect2_setup_backend_library(freenect2_opencl)

target_link_libraries(freenect2_opencl
  PRIVATE freenect2_support freenect2_logging
)

if(VCPKG_MANIFEST_MODE)
  target_link_libraries(freenect2_opencl
    PRIVATE OpenCL::OpenCL OpenCL::HeadersCpp spdlog::spdlog
  )
else()
  target_include_directories(freenect2_opencl PRIVATE ${OpenCL_INCLUDE_DIRS})
  target_link_libraries(freenect2_opencl
    PRIVATE ${OpenCL_LIBRARIES} spdlog::spdlog
  )
endif()
