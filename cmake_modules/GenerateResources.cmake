FUNCTION(GENERATE_RESOURCES OUTPUT BASE_FOLDER)

ADD_EXECUTABLE(generate_resources_tool
  ${PROJECT_SOURCE_DIR}/tools/generate_resources.cpp
)

ADD_CUSTOM_COMMAND(
  OUTPUT ${OUTPUT}
  COMMAND generate_resources_tool ${BASE_FOLDER} ${ARGN} > ${OUTPUT}
  WORKING_DIRECTORY ${BASE_FOLDER}
  DEPENDS generate_resources_tool ${ARGN}
)

ENDFUNCTION(GENERATE_RESOURCES)

# Modern alias — used by src/support, src/opencl, src/opengl CMakeLists.txt.
# Signature: generate_resources_target(<OUTPUT_VAR> <NAME_SUFFIX> <RESOURCE_FILE...>)
#
# Each caller passes only the resource files IT embeds, so a consumer's
# resources.inc.h (and the DLL it's compiled into) only contains what that
# consumer actually needs — e.g. freenect2_opencl doesn't end up embedding
# OpenGL shader source, and vice versa. NAME_SUFFIX disambiguates the output
# file and custom target across the multiple call sites (the underlying
# generate_resources_tool executable is shared, built once).
FUNCTION(GENERATE_RESOURCES_TARGET OUTPUT_VAR NAME_SUFFIX)
  set(RESOURCES ${ARGN})

  set(RESOURCES_INC_FILE "${PROJECT_BINARY_DIR}/resources_${NAME_SUFFIX}.inc.h")

  if(NOT TARGET generate_resources_tool)
    ADD_EXECUTABLE(generate_resources_tool
      ${PROJECT_SOURCE_DIR}/tools/generate_resources.cpp
    )
  endif()

  # Build absolute paths for DEPENDS
  set(RESOURCES_DEPS)
  foreach(RES IN LISTS RESOURCES)
    list(APPEND RESOURCES_DEPS "${PROJECT_SOURCE_DIR}/${RES}")
  endforeach()

  ADD_CUSTOM_COMMAND(
    OUTPUT ${RESOURCES_INC_FILE}
    COMMAND generate_resources_tool ${PROJECT_SOURCE_DIR} ${RESOURCES} > ${RESOURCES_INC_FILE}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS generate_resources_tool ${RESOURCES_DEPS}
    COMMENT "Generating resource include file (${NAME_SUFFIX})"
    VERBATIM
  )

  # Create a custom target to ensure the resource file is built
  ADD_CUSTOM_TARGET(resources_generator_${NAME_SUFFIX} DEPENDS ${RESOURCES_INC_FILE})

  set(${OUTPUT_VAR} ${RESOURCES_INC_FILE} PARENT_SCOPE)
ENDFUNCTION(GENERATE_RESOURCES_TARGET)
