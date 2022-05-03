cmake_minimum_required(VERSION 3.16.3)
project(proc_image_processing)

# Use optimized build by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

message("##### - Build type: ${CMAKE_BUILD_TYPE} - #####")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(project_path ${CMAKE_CURRENT_SOURCE_DIR})
set(proc_image_processing_SRC_DIR "src/")
set(proc_image_processing_CPU_SRC_DIR "src/proc_image_processing/cpu")
set(proc_image_processing_GPU_SRC_DIR "src/proc_image_processing/gpu")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs image_transport cv_bridge roslaunch pcl_ros sonia_common)
find_package(CUDA)
if (CUDA_FOUND)
    include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
            ${CMAKE_BINARY_DIR} ${project_path}/tools/detect_gpu.c
            CMAKE_FLAGS
            -DINCLUDE_DIRECTORIES=${CUDA_TOOLKIT_INCLUDE}
            -DLINK_LIBRARIES=${CUDA_CUDART_LIBRARY}
            COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
            RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR
            )
    if (COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
        message("##### - ${RUN_OUTPUT_VAR} - #####")
    else ()
        message("##### - ${RUN_OUTPUT_VAR} Build anyway since CUDA is present. - #####")
    endif ()

    message("##### - Using GPU mode - #####")

    # GPU Mode, use custom OpenCV
    set(OpenCV_DIR /usr/local)
    find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
    include_directories(${OpenCV_INCLUDE_DIRS})

    # Add Benchmarking tools
    add_subdirectory(tools/)

    # Set source files
    file(
            GLOB_RECURSE
            proc_image_processing_SRC_FILES
            "${proc_image_processing_GPU_SRC_DIR}/*.cc"
            "${proc_image_processing_GPU_SRC_DIR}/*.h"
            "${proc_image_processing_CPU_SRC_DIR}/*.cc"
            "${proc_image_processing_CPU_SRC_DIR}/*.h"
    )
    set(main ${project_path}/${proc_image_processing_GPU_SRC_DIR}/main.cc)
else ()
    message("##### - Using CPU mode, CUDA not found - #####")

    # CPU Mode, use default OpenCv
    find_package(OpenCV REQUIRED)

    # Set source files
    file(
            GLOB_RECURSE
            proc_image_processing_SRC_FILES
            "${proc_image_processing_CPU_SRC_DIR}/*.cc"
            "${proc_image_processing_CPU_SRC_DIR}/*.h"
    )
    set(main ${project_path}/${proc_image_processing_CPU_SRC_DIR}/main.cc)
endif (CUDA_FOUND)

# TODO fix major_edge_extractor implementation for GPU mode. Likely due to incompatibility.
list(REMOVE_ITEM proc_image_processing_SRC_FILES ${project_path}/${proc_image_processing_CPU_SRC_DIR}/algorithm/major_edge_extractor.h)
list(REMOVE_ITEM proc_image_processing_SRC_FILES ${project_path}/${proc_image_processing_CPU_SRC_DIR}/algorithm/major_edge_extractor.cc)

# Exclude mains
list(REMOVE_ITEM proc_image_processing_SRC_FILES ${project_path}/${proc_image_processing_CPU_SRC_DIR}/main.cc)
list(REMOVE_ITEM proc_image_processing_SRC_FILES ${project_path}/${proc_image_processing_CPU_SRC_DIR}/republisher.cc)
list(REMOVE_ITEM proc_image_processing_SRC_FILES ${project_path}/${proc_image_processing_GPU_SRC_DIR}/main.cc)

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

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_image_processing_SRC_DIR}
        ${sonia_common_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}_node
        ${main}
        ${proc_image_processing_SRC_FILES}
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${pcl_ros_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif ()

add_executable(republisher_node ${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/republisher.cc)
target_link_libraries(republisher_node ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
add_dependencies(republisher_node ${catkin_EXPORTED_TARGETS})