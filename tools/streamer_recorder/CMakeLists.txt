# tools/streamer_recorder/CMakeLists.txt — ProtonectSR streaming/recording tool

message(STATUS "Configuring streamer_recorder")

# In-tree vs out-of-tree
if(TARGET freenect2)
  message(STATUS "  Using in-tree freenect2 target")
else()
  find_package(freenect2 REQUIRED)
endif()

# Find OpenCV (required)
find_package(OpenCV REQUIRED)

add_executable(ProtonectSR
  ProtonectSR.cpp
  PracticalSocket.cpp
  streamer.cpp
  recorder.cpp
)

target_compile_features(ProtonectSR PRIVATE cxx_std_17)
target_include_directories(ProtonectSR PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../examples
)
target_link_libraries(ProtonectSR PRIVATE freenect2 ${OpenCV_LIBS})

if(WIN32)
  target_link_libraries(ProtonectSR PRIVATE ws2_32)
endif()

# OpenGL viewer support
if(ENABLE_OPENGL)
  if(TARGET freenect2)
    if(TARGET OpenGL::GL)
      set(OPENGL_AVAILABLE TRUE)
    endif()
  else()
    find_package(OpenGL QUIET)
    if(NOT TARGET freenect2)
      list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_modules")
      find_package(PkgConfig QUIET)
      find_package(GLFW3 QUIET)
    endif()
    if(GLFW3_FOUND AND OPENGL_FOUND)
      set(OPENGL_AVAILABLE TRUE)
    endif()
  endif()

  if(OPENGL_AVAILABLE)
    target_sources(ProtonectSR PRIVATE
      ../../examples/viewer.cpp
      ../../src/opengl/flextGL.cpp
    )
    target_compile_definitions(ProtonectSR PRIVATE EXAMPLES_WITH_OPENGL_SUPPORT=1)
    target_include_directories(ProtonectSR PRIVATE ${PROJECT_SOURCE_DIR}/src/opengl)
    target_link_libraries(ProtonectSR PRIVATE OpenGL::GL)

    if(TARGET glfw)
      target_link_libraries(ProtonectSR PRIVATE glfw)
    elseif(DEFINED GLFW3_LIBRARIES)
      target_link_libraries(ProtonectSR PRIVATE ${GLFW3_LIBRARIES})
    endif()
  endif()
endif()

# Install
include(GNUInstallDirs)
install(TARGETS ProtonectSR DESTINATION ${CMAKE_INSTALL_BINDIR})
