cmake_minimum_required(VERSION 3.10)
project(proc_image_processing)

set(CMAKE_CXX_STANDARD 11)

set(proc_image_processing_SRC_DIR "src")

set(CMAKE_FIND_PACKAGE_RESOLVE_SYMLINKS TRUE)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CUDA_NVCC_EXECUTABLE "/usr/local/cuda/bin/nvcc")
set(CUDA_CUDART_LIBRARY "/usr/local/cuda/lib64/libcudart.so")
find_package(CUDA)
if(CUDA_FOUND)
  try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
          ${CMAKE_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/tools/detect_gpu.c
          CMAKE_FLAGS
          -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
          -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
          COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
          RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
  message("${RUN_OUTPUT_VAR}") # Display number of GPUs found
  # COMPILE_RESULT_VAR is TRUE when compile succeeds
  # RUN_RESULT_VAR is zero when a GPU is found
  if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
    set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
    message(INFO " - CUDA GPU found, using GPU image filters library")
    include_directories(${CUDA_TOOLKIT_INCLUDE_DIRS})
    # TODO LINK CUDA LIBRARY OF FILTERS
  else()
    set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    # TODO LINK STANDARD LIBRARY OF FILTERS
    message(INFO " - CUDA GPU not found, using standard image filters library")
    unset(CUDA_TOOLKIT_ROOT_DIR)
    unset(CUDA_NVCC_EXECUTABLE)
    unset(CUDA_CUDART_LIBRARY)
  endif()
endif(CUDA_FOUND)

set(OpenCV_DIR /usr/local)
find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        roslaunch
        pcl_ros
        sonia_common
        )

catkin_package(
        INCLUDE_DIRS ${proc_image_processing_SRC_DIR}
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        roslaunch
        pcl_ros
        sonia_common
)

file(GLOB_RECURSE proc_image_processing_FILES
        "${proc_image_processing_SRC_DIR}/*.cc"
        "${proc_image_processing_SRC_DIR}/*.h")

list(REMOVE_ITEM proc_image_processing_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/main.cc)
list(REMOVE_ITEM proc_image_processing_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/uncompress/republisher.cc)
list(REMOVE_ITEM proc_image_processing_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/algorithm/major_edge_extractor.cc)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_image_processing_SRC_DIR}
        ${sonia_common_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node ${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/main.cc ${proc_image_processing_FILES} src/proc_image_processing/filters/square_detection.h src/proc_image_processing/filters/white_filter.h)
target_link_libraries(proc_image_processing_node
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${pcl_ros_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        )
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

add_executable(republisher_node ${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/uncompress/republisher.cc ${proc_image_processing_FILES})
target_link_libraries(republisher_node
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${pcl_ros_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        )
add_dependencies(republisher_node ${catkin_EXPORTED_TARGETS})

#add_executable(proc_image_processing ${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/main.cc)
#target_link_libraries(proc_image_processing ${OpenCV_LIBS})