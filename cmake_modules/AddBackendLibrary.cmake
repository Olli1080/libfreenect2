# cmake_modules/AddBackendLibrary.cmake
#
# Shared setup for freenect2 backend DLLs (opencl, opengl, cuda, turbojpeg,
# vaapi, vt): standard include directories, C++17, hidden visibility, an
# export header, and an install rule. Previously each backend's CMakeLists.txt
# hand-rolled this ~10-line block identically; factored here so the export
# convention can only drift by editing this one function, not six copies.
#
# Callers still declare their own target_sources()/target_link_libraries()
# and any backend-specific settings (e.g. CUDA's CUDA_ARCHITECTURES/nvcc
# flags, VAAPI's extra include dir) after calling this.
function(freenect2_setup_backend_library TARGET_NAME)
  target_include_directories(${TARGET_NAME}
    PRIVATE
      ${PROJECT_SOURCE_DIR}/include
      ${PROJECT_SOURCE_DIR}/include/internal
      ${PROJECT_BINARY_DIR}
      ${PROJECT_SOURCE_DIR}/src
  )

  target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
  set_target_properties(${TARGET_NAME} PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
  )

  include(GenerateExportHeader)
  generate_export_header(${TARGET_NAME}
    BASE_NAME ${TARGET_NAME}
    EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/${TARGET_NAME}/export.h
  )

  # EXPORT freenect2Targets: joins the same export set core freenect2 uses (see the root
  # CMakeLists' install(EXPORT freenect2Targets ...)) so freenect2Targets.cmake carries a real,
  # per-config-correct imported target for every backend - harmless for the backends with only
  # PRIVATE include dirs (nothing here to export), and required for freenect2_audio, the one
  # backend meant to be linked directly by external consumers.
  install(TARGETS ${TARGET_NAME}
    EXPORT freenect2Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endfunction()
