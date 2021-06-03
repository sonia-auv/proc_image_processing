cmake_minimum_required(VERSION 3.10)
project(proc_image_processing_benchmark)

set(CMAKE_CXX_STANDARD 17)

set(proc_image_processing_benchmark_SRC_DIR "benchmarks")
find_package(Boost COMPONENTS program_options filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

IF(NOT CUDA_FOUND)
#    set(CMAKE_FIND_PACKAGE_RESOLVE_SYMLINKS TRUE)
#    set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
#    set(CUDA_NVCC_EXECUTABLE "/usr/local/cuda/bin/nvcc")
#    set(CUDA_CUDART_LIBRARY "/usr/local/cuda/lib64/libcudart.so")
    find_package(CUDA REQUIRED)
    include_directories(${CUDA_TOOLKIT_INCLUDE_DIRS})
endif()

set(OpenCV_DIR /usr/local)
find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(benchmark1 benchmarks/benchmark1.cc benchmarks/tools.h benchmarks/tools.cc)
target_link_libraries(benchmark1 ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(benchmark2 benchmarks/benchmark2.cc benchmarks/tools.h benchmarks/tools.cc)
target_link_libraries(benchmark2 ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(benchmark3 benchmarks/benchmark3.cc benchmarks/tools.h benchmarks/tools.cc)
target_link_libraries(benchmark3 ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})