# src/opengl/CMakeLists.txt — OpenGL depth processing + viewer backend

add_library(freenect2_opengl)

target_sources(freenect2_opengl PRIVATE
  flextGL.cpp
  opengl_depth_packet_processor.cpp
  ${PROJECT_SOURCE_DIR}/src/resource.cpp
)

# --- Resource Embedding ---
# Only this backend's own GLSL shader source — see src/support/CMakeLists.txt
# for why resources are scoped per-consumer instead of one shared table.
generate_resources_target(RESOURCES_INC_FILE opengl
  src/opengl/shader/debug.fs
  src/opengl/shader/default.vs
  src/opengl/shader/filter1.fs
  src/opengl/shader/filter2.fs
  src/opengl/shader/stage1.fs
  src/opengl/shader/stage2.fs
)
target_sources(freenect2_opengl PRIVATE ${RESOURCES_INC_FILE})
target_compile_definitions(freenect2_opengl PRIVATE RESOURCES_INC)

freenect2_setup_backend_library(freenect2_opengl)

target_link_libraries(freenect2_opengl
  PRIVATE freenect2_support freenect2_logging
)

if(VCPKG_MANIFEST_MODE)
  target_link_libraries(freenect2_opengl
    PRIVATE glfw OpenGL::GL spdlog::spdlog
  )
else()
  target_include_directories(freenect2_opengl PRIVATE ${GLFW3_INCLUDE_DIRS})
  target_link_libraries(freenect2_opengl
    PRIVATE ${GLFW3_LIBRARIES} ${OPENGL_gl_LIBRARY} spdlog::spdlog
  )
endif()
