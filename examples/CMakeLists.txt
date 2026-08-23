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

