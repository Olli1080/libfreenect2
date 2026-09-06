# src/cuda/CMakeLists.txt — CUDA depth processing backend
# Separate sub-library following REACH module pattern.

add_library(freenect2_cuda)

target_sources(freenect2_cuda
  PRIVATE
    cuda_depth_packet_processor.cu
    cuda_depth_packet_processor.cpp
    #
    cuda_kde_depth_packet_processor.cu
    cuda_kde_depth_packet_processor.cpp
  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${PROJECT_SOURCE_DIR}/include/"
  FILES
    "${PROJECT_SOURCE_DIR}/include/internal/cuda/cuda_depth_packet_processor.h"
    "${PROJECT_SOURCE_DIR}/include/internal/cuda/cuda_kde_depth_packet_processor.h"
)

freenect2_setup_backend_library(freenect2_cuda)
set_target_properties(freenect2_cuda PROPERTIES CUDA_ARCHITECTURES "native")

# nvcc flags
target_compile_options(freenect2_cuda PRIVATE "$<$<COMPILE_LANGUAGE:CUDA>:-use_fast_math>")
if(NOT MSVC)
  target_compile_options(freenect2_cuda PRIVATE "$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler;-fPIC>")
endif()
if(MSVC)
  target_compile_options(freenect2_cuda PRIVATE "$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/utf-8>")
  target_compile_definitions(freenect2_cuda PRIVATE "$<$<COMPILE_LANGUAGE:CUDA>:FMT_UNICODE=0>")
endif()
target_compile_options(freenect2_cuda PRIVATE "$<$<COMPILE_LANGUAGE:CUDA>:-std=c++17>")

target_link_libraries(freenect2_cuda
  PRIVATE CUDA::cudart freenect2_support freenect2_logging
)
