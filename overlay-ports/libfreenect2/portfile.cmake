# Local overlay port: builds directly from this fork's working tree
# (Olli1080/libfreenect2), not from a GitHub release tag. There is
# deliberately no vcpkg_from_github/SHA512 pin here -- this port always
# tracks whatever is currently checked out two directories up.
get_filename_component(SOURCE_PATH "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        opengl     ENABLE_OPENGL
        opencl     ENABLE_OPENCL
        openni2    BUILD_OPENNI2_DRIVER
        audio      ENABLE_AUDIO
        vulkan     ENABLE_VULKAN
        python     BUILD_PYTHON_BINDINGS
)

vcpkg_find_acquire_program(PKGCONFIG)

if (NOT VCPKG_BUILD_TYPE OR VCPKG_BUILD_TYPE STREQUAL "debug")
    set(path_suffix "/debug")
endif()
if (NOT VCPKG_BUILD_TYPE OR VCPKG_BUILD_TYPE STREQUAL "release")
    set(path_suffix "")
endif()
vcpkg_backup_env_variables(VARS PKG_CONFIG_PATH)
vcpkg_host_path_list(PREPEND ENV{PKG_CONFIG_PATH} "${CURRENT_INSTALLED_DIR}${path_suffix}/lib/pkgconfig")

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        "-DPKG_CONFIG_EXECUTABLE=${PKGCONFIG}"
        -DENABLE_CUDA=OFF
        -DENABLE_VAAPI=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_STREAMER_RECORDER=OFF
        -DBUILD_PCL_EXAMPLE=OFF
        -DBUILD_ONNX_EXAMPLES=OFF
        -DDOWNLOAD_ONNX_MODELS=OFF
        ${FEATURE_OPTIONS}
)
vcpkg_restore_env_variables(VARS PKG_CONFIG_PATH)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(PACKAGE_NAME freenect2 CONFIG_PATH lib/cmake/freenect2)

vcpkg_copy_pdbs()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/GPL2" "${SOURCE_PATH}/APACHE20")

vcpkg_fixup_pkgconfig()
