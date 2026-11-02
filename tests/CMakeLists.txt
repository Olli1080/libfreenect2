# tests/CMakeLists.txt — library tests that need no Kinect

add_executable(pipeline_swap_test pipeline_swap_test.cpp)
target_link_libraries(pipeline_swap_test PRIVATE freenect2)
add_test(NAME pipeline_swap_test COMMAND pipeline_swap_test)
