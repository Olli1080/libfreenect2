# src/vulkan/CMakeLists.txt — Vulkan depth processing backend
#
# Phase 2 (see the "Add a Vulkan depth-processing backend" plan): all 4
# depth-decode stages are compiled/wired and chained with barriers.

add_library(freenect2_vulkan)

target_sources(freenect2_vulkan PRIVATE
  vulkan_depth_packet_processor.cpp
)

# --- Shader Compilation (GLSL -> SPIR-V) ---
find_program(GLSLC_EXECUTABLE glslc HINTS ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/tools/shaderc)
if(NOT GLSLC_EXECUTABLE)
  message(FATAL_ERROR "glslc (from vcpkg shaderc) not found -- required to compile Vulkan compute shaders")
endif()

set(VULKAN_SHADER_SOURCES
  process_pixel_stage1.comp
  filter_pixel_stage1.comp
  process_pixel_stage2.comp
  filter_pixel_stage2.comp
)
set(VULKAN_SHADER_SPV_FILES)
set(VULKAN_SHADER_SPV_NAMES)

foreach(SHADER_SRC IN LISTS VULKAN_SHADER_SOURCES)
  set(SHADER_SPV_NAME "${SHADER_SRC}.spv")
  set(SHADER_SPV "${CMAKE_CURRENT_BINARY_DIR}/${SHADER_SPV_NAME}")
  add_custom_command(
    OUTPUT ${SHADER_SPV}
    COMMAND ${GLSLC_EXECUTABLE} -o ${SHADER_SPV} ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_SRC}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_SRC}
    COMMENT "Compiling Vulkan shader ${SHADER_SRC} -> SPIR-V"
    VERBATIM
  )
  list(APPEND VULKAN_SHADER_SPV_FILES ${SHADER_SPV})
  list(APPEND VULKAN_SHADER_SPV_NAMES ${SHADER_SPV_NAME})
endforeach()

add_custom_target(freenect2_vulkan_shaders DEPENDS ${VULKAN_SHADER_SPV_FILES})
add_dependencies(freenect2_vulkan freenect2_vulkan_shaders)

# --- Resource Embedding ---
# Compiled SPIR-V binaries live in the build tree, not the source tree, so
# this can't use generate_resources_target() (src/support/CMakeLists.txt's
# shared helper, which only knows about PROJECT_SOURCE_DIR paths) --
# reuses the same underlying generate_resources_tool directly instead, same
# embedding mechanism as every other backend's shader/kernel source.
if(NOT TARGET generate_resources_tool)
  add_executable(generate_resources_tool ${PROJECT_SOURCE_DIR}/tools/generate_resources.cpp)
endif()

set(VULKAN_RESOURCES_INC_FILE "${CMAKE_CURRENT_BINARY_DIR}/resources_vulkan.inc.h")
get_filename_component(VULKAN_SHADER_SPV_DIR "${CMAKE_CURRENT_BINARY_DIR}" ABSOLUTE)

add_custom_command(
  OUTPUT ${VULKAN_RESOURCES_INC_FILE}
  COMMAND generate_resources_tool ${VULKAN_SHADER_SPV_DIR} ${VULKAN_SHADER_SPV_NAMES} > ${VULKAN_RESOURCES_INC_FILE}
  WORKING_DIRECTORY ${VULKAN_SHADER_SPV_DIR}
  DEPENDS generate_resources_tool ${VULKAN_SHADER_SPV_FILES}
  COMMENT "Generating resource include file (vulkan)"
  VERBATIM
)
add_custom_target(resources_generator_vulkan DEPENDS ${VULKAN_RESOURCES_INC_FILE})
add_dependencies(freenect2_vulkan resources_generator_vulkan)

target_sources(freenect2_vulkan PRIVATE ${VULKAN_RESOURCES_INC_FILE})
target_compile_definitions(freenect2_vulkan PRIVATE RESOURCES_INC)

freenect2_setup_backend_library(freenect2_vulkan)

# So vulkan_depth_packet_processor.cpp's #include "resources_vulkan.inc.h"
# (self-contained loader, see its file-level comment) can find the file --
# it's generated into this target's own binary dir, which
# freenect2_setup_backend_library doesn't add by default.
target_include_directories(freenect2_vulkan PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(freenect2_vulkan
  PRIVATE freenect2_support freenect2_logging Vulkan::Vulkan Vulkan::Headers spdlog::spdlog
)
