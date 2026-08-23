# examples/CMakeLists.txt — Protonect reference application

message(STATUS "Configuring examples")

# In-tree vs out-of-tree
if(TARGET freenect2)
  message(STATUS "  Using in-tree freenect2 target")
else()
  find_package(freenect2 REQUIRED)
endif()

add_executable(Protonect Protonect.cpp)

target_compile_features(Protonect PRIVATE cxx_std_17)
target_link_libraries(Protonect PRIVATE freenect2)

# OpenGL viewer support
if(ENABLE_OPENGL)
  if(TARGET freenect2)
    # In-tree build: OpenGL deps already resolved by parent
    if(TARGET OpenGL::GL)
      set(OPENGL_AVAILABLE TRUE)
    endif()
  else()
    find_package(OpenGL QUIET)
    if(NOT TARGET freenect2)
      list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules")
      find_package(PkgConfig QUIET)
      find_package(GLFW3 QUIET)
    endif()
    if(GLFW3_FOUND AND OPENGL_FOUND)
      set(OPENGL_AVAILABLE TRUE)
    endif()
  endif()

  if(OPENGL_AVAILABLE)
    target_sources(Protonect PRIVATE viewer.cpp ../src/opengl/flextGL.cpp)
    target_compile_definitions(Protonect PRIVATE EXAMPLES_WITH_OPENGL_SUPPORT=1)
    target_include_directories(Protonect PRIVATE ${PROJECT_SOURCE_DIR}/src/opengl)
    target_link_libraries(Protonect PRIVATE OpenGL::GL)

    if(TARGET glfw)
      target_link_libraries(Protonect PRIVATE glfw)
    elseif(DEFINED GLFW3_LIBRARIES)
      target_link_libraries(Protonect PRIVATE ${GLFW3_LIBRARIES})
    endif()
  endif()
endif()

# Install
include(GNUInstallDirs)
install(TARGETS Protonect DESTINATION ${CMAKE_INSTALL_BINDIR})

# --- Audio Capture Example ---
if(TARGET freenect2_audio)
  add_executable(AudioCapture AudioCapture.cpp)
  target_compile_features(AudioCapture PRIVATE cxx_std_17)
  target_link_libraries(AudioCapture PRIVATE freenect2_audio)

  install(TARGETS AudioCapture DESTINATION ${CMAKE_INSTALL_BINDIR})

  # --- Audio/Video Correlation Example ---
  add_executable(AudioVideoSync AudioVideoSync.cpp)
  target_compile_features(AudioVideoSync PRIVATE cxx_std_17)
  target_link_libraries(AudioVideoSync PRIVATE freenect2 freenect2_audio)

  install(TARGETS AudioVideoSync DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# --- Vulkan Backend Phase 1 Verification Tool ---
# Not a permanent example app -- see "Add a Vulkan depth-processing backend"
# plan. Compares VulkanDepthPacketProcessor's stage-1 output against
# OpenCLDepthPacketProcessor's for the same real captured raw depth packet.
if(TARGET freenect2_vulkan AND TARGET freenect2_opencl)
  add_executable(VulkanStage1Verify VulkanStage1Verify.cpp)
  target_compile_features(VulkanStage1Verify PRIVATE cxx_std_17)
  target_include_directories(VulkanStage1Verify PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/internal
    ${PROJECT_BINARY_DIR}/freenect2_opencl
    ${PROJECT_BINARY_DIR}/freenect2_vulkan
  )
  target_link_libraries(VulkanStage1Verify PRIVATE freenect2 freenect2_opencl freenect2_vulkan)
endif()

# --- PCL Point Cloud Example ---
if(BUILD_PCL_EXAMPLE)
  find_package(PCL REQUIRED COMPONENTS common visualization)

  add_executable(PCLPointCloud PCLPointCloud.cpp)
  target_compile_features(PCLPointCloud PRIVATE cxx_std_17)
  target_include_directories(PCLPointCloud PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(PCLPointCloud PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(PCLPointCloud PRIVATE freenect2 ${PCL_LIBRARIES})

  install(TARGETS PCLPointCloud DESTINATION ${CMAKE_INSTALL_BINDIR})

  # --- PCL Point Cloud + Direction-of-Arrival Example ---
  if(TARGET freenect2_audio)
    add_executable(PCLPointCloudAudio PCLPointCloudAudio.cpp)
    target_compile_features(PCLPointCloudAudio PRIVATE cxx_std_17)
    target_include_directories(PCLPointCloudAudio PRIVATE ${PCL_INCLUDE_DIRS})
    target_compile_definitions(PCLPointCloudAudio PRIVATE ${PCL_DEFINITIONS})
    target_link_libraries(PCLPointCloudAudio PRIVATE freenect2 freenect2_audio ${PCL_LIBRARIES})

    install(TARGETS PCLPointCloudAudio DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

# --- ONNX Runtime Object Detection / Pose Estimation Examples ---
if(BUILD_ONNX_EXAMPLES)
  find_package(onnxruntime CONFIG REQUIRED)
  # Scoped to just what's used (core/imgproc/highgui), not the full default
  # component set -- smaller link, and one fewer thing pulled into a binary
  # that's already linking a large onnxruntime.dll. Note this does NOT fix
  # the "Schema error: ... already registered" spam both examples print at
  # startup -- verified that's onnxruntime.dll itself double-registering its
  # own bundled ONNX operator schemas internally (every "already registered
  # from" points at the exact same file/line as the "registering" one), not
  # anything caused by OpenCV or this project. Harmless; both examples run
  # correctly despite it. Not fixable from this CMakeLists.txt -- it's a
  # vcpkg onnxruntime packaging artifact.
  find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

  add_library(onnx_common OBJECT onnx_common.cpp)
  target_compile_features(onnx_common PUBLIC cxx_std_17)
  target_link_libraries(onnx_common PUBLIC onnxruntime::onnxruntime ${OpenCV_LIBS})

  add_executable(ONNXObjectDetection ONNXObjectDetection.cpp)
  target_compile_features(ONNXObjectDetection PRIVATE cxx_std_17)
  target_link_libraries(ONNXObjectDetection PRIVATE freenect2 onnx_common)
  install(TARGETS ONNXObjectDetection DESTINATION ${CMAKE_INSTALL_BINDIR})

  add_executable(ONNXPoseEstimation ONNXPoseEstimation.cpp)
  target_compile_features(ONNXPoseEstimation PRIVATE cxx_std_17)
  target_link_libraries(ONNXPoseEstimation PRIVATE freenect2 onnx_common)
  install(TARGETS ONNXPoseEstimation DESTINATION ${CMAKE_INSTALL_BINDIR})

  # --- Optional: auto-download official Ultralytics .pt weights and export
  # them to ONNX via a local Python venv. Off by default -- this pulls in a
  # full Python + PyTorch toolchain (~2GB), which is a lot to impose on
  # everyone who just wants to build the examples. Output goes to
  # examples/models/ (gitignored, not committed) so `--model
  # examples/models/yolov8n.onnx` / `yolov8n-pose.onnx` works after this runs.
  if(DOWNLOAD_ONNX_MODELS)
    set(ONNX_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models")
    file(MAKE_DIRECTORY "${ONNX_MODELS_DIR}")

    set(YOLO_RELEASE_BASE "https://github.com/ultralytics/assets/releases/download/v8.3.0")

    if(NOT EXISTS "${ONNX_MODELS_DIR}/yolov8n.pt")
      message(STATUS "Downloading yolov8n.pt (official Ultralytics release v8.3.0, ~6.2 MB)...")
      file(DOWNLOAD "${YOLO_RELEASE_BASE}/yolov8n.pt" "${ONNX_MODELS_DIR}/yolov8n.pt"
        EXPECTED_HASH SHA256=f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36
        STATUS _yolo_dl_status)
      list(GET _yolo_dl_status 0 _yolo_dl_code)
      if(NOT _yolo_dl_code EQUAL 0)
        list(GET _yolo_dl_status 1 _yolo_dl_msg)
        file(REMOVE "${ONNX_MODELS_DIR}/yolov8n.pt")
        message(WARNING "Failed to download yolov8n.pt: ${_yolo_dl_msg}")
      endif()
    endif()

    if(NOT EXISTS "${ONNX_MODELS_DIR}/yolov8n-pose.pt")
      message(STATUS "Downloading yolov8n-pose.pt (official Ultralytics release v8.3.0, ~6.5 MB)...")
      file(DOWNLOAD "${YOLO_RELEASE_BASE}/yolov8n-pose.pt" "${ONNX_MODELS_DIR}/yolov8n-pose.pt"
        EXPECTED_HASH SHA256=c6fa93dd1ee4a2c18c900a45c1d864a1c6f7aba75d84f91648a30b7fb641d212
        STATUS _yolo_pose_dl_status)
      list(GET _yolo_pose_dl_status 0 _yolo_pose_dl_code)
      if(NOT _yolo_pose_dl_code EQUAL 0)
        list(GET _yolo_pose_dl_status 1 _yolo_pose_dl_msg)
        file(REMOVE "${ONNX_MODELS_DIR}/yolov8n-pose.pt")
        message(WARNING "Failed to download yolov8n-pose.pt: ${_yolo_pose_dl_msg}")
      endif()
    endif()

    find_package(Python3 COMPONENTS Interpreter)
    if(NOT Python3_FOUND)
      message(WARNING "DOWNLOAD_ONNX_MODELS needs Python 3 to export .pt -> .onnx (via a local venv + the "
                       "ultralytics pip package) -- none found. Install Python 3 and reconfigure, or export "
                       "manually: pip install ultralytics && yolo export model=${ONNX_MODELS_DIR}/yolov8n.pt format=onnx")
    else()
      set(ONNX_EXPORT_VENV "${CMAKE_BINARY_DIR}/onnx_export_venv")
      if(WIN32)
        set(ONNX_VENV_PYTHON "${ONNX_EXPORT_VENV}/Scripts/python.exe")
      else()
        set(ONNX_VENV_PYTHON "${ONNX_EXPORT_VENV}/bin/python")
      endif()

      if(NOT EXISTS "${ONNX_VENV_PYTHON}")
        message(STATUS "Creating Python venv for YOLOv8 ONNX export at ${ONNX_EXPORT_VENV}...")
        execute_process(
          COMMAND "${Python3_EXECUTABLE}" -m venv "${ONNX_EXPORT_VENV}"
          RESULT_VARIABLE _onnx_venv_result
        )
        if(NOT _onnx_venv_result EQUAL 0)
          message(WARNING "Failed to create Python venv for ONNX export (exit ${_onnx_venv_result})")
          set(ONNX_VENV_PYTHON "")
        endif()
      endif()

      if(ONNX_VENV_PYTHON AND EXISTS "${ONNX_VENV_PYTHON}")
        if(NOT EXISTS "${ONNX_MODELS_DIR}/yolov8n.onnx" OR NOT EXISTS "${ONNX_MODELS_DIR}/yolov8n-pose.onnx")
          message(STATUS "Installing ultralytics (+ PyTorch, ~2GB) into the export venv -- this can take a while...")
          execute_process(COMMAND "${ONNX_VENV_PYTHON}" -m pip install --quiet --upgrade pip)
          execute_process(
            COMMAND "${ONNX_VENV_PYTHON}" -m pip install --quiet ultralytics
            RESULT_VARIABLE _onnx_pip_result
          )
          if(NOT _onnx_pip_result EQUAL 0)
            message(WARNING "Failed to pip install ultralytics into the export venv (exit ${_onnx_pip_result})")
          endif()
        endif()

        if(EXISTS "${ONNX_MODELS_DIR}/yolov8n.pt" AND NOT EXISTS "${ONNX_MODELS_DIR}/yolov8n.onnx")
          message(STATUS "Exporting yolov8n.pt -> yolov8n.onnx...")
          execute_process(
            COMMAND "${ONNX_VENV_PYTHON}" -c "from ultralytics import YOLO; YOLO(r'${ONNX_MODELS_DIR}/yolov8n.pt').export(format='onnx')"
            WORKING_DIRECTORY "${ONNX_MODELS_DIR}"
            RESULT_VARIABLE _onnx_export_result
          )
          if(NOT _onnx_export_result EQUAL 0)
            message(WARNING "Failed to export yolov8n.onnx (exit ${_onnx_export_result})")
          endif()
        endif()

        if(EXISTS "${ONNX_MODELS_DIR}/yolov8n-pose.pt" AND NOT EXISTS "${ONNX_MODELS_DIR}/yolov8n-pose.onnx")
          message(STATUS "Exporting yolov8n-pose.pt -> yolov8n-pose.onnx...")
          execute_process(
            COMMAND "${ONNX_VENV_PYTHON}" -c "from ultralytics import YOLO; YOLO(r'${ONNX_MODELS_DIR}/yolov8n-pose.pt').export(format='onnx')"
            WORKING_DIRECTORY "${ONNX_MODELS_DIR}"
            RESULT_VARIABLE _onnx_export_pose_result
          )
          if(NOT _onnx_export_pose_result EQUAL 0)
            message(WARNING "Failed to export yolov8n-pose.onnx (exit ${_onnx_export_pose_result})")
          endif()
        endif()

        if(EXISTS "${ONNX_MODELS_DIR}/yolov8n.onnx" AND EXISTS "${ONNX_MODELS_DIR}/yolov8n-pose.onnx")
          message(STATUS "ONNX models ready:")
          message(STATUS "  ${ONNX_MODELS_DIR}/yolov8n.onnx")
          message(STATUS "  ${ONNX_MODELS_DIR}/yolov8n-pose.onnx")
        endif()
      endif()
    endif()
  endif()
endif()

