cmake_minimum_required(VERSION 3.10)
project(proc_image_processing)

set(CMAKE_CXX_STANDARD 17)

set(proc_image_processing_SRC_DIR "src")
set(proc_image_processing_CPU_SRC_DIR "src/proc_image_processing/cpu")
set(proc_image_processing_GPU_SRC_DIR "src/proc_image_processing/gpu")

function(cpu_mode)
    find_package(OpenCV REQUIRED)
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

    file(GLOB_RECURSE proc_image_processing_CPU_FILES "${proc_image_processing_CPU_SRC_DIR}/*.cc" "${proc_image_processing_CPU_SRC_DIR}/*.h")

    list(REMOVE_ITEM proc_image_processing_CPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_CPU_SRC_DIR}/main.cc)
    list(REMOVE_ITEM proc_image_processing_CPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_CPU_SRC_DIR}/algorithm/major_edge_extractor.cc)
    list(REMOVE_ITEM proc_image_processing_CPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_CPU_SRC_DIR}/algorithm/major_edge_extractor.h)

    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${proc_image_processing_SRC_DIR}
            ${sonia_common_INCLUDE_DIRS}
            ${pcl_ros_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )

    add_executable(
            ${PROJECT_NAME}_node
            ${proc_image_processing_CPU_SRC_DIR}/main.cc
            ${proc_image_processing_CPU_FILES}
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
endfunction()

function(gpu_mode)
    set(OpenCV_DIR /usr/local)
    find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_subdirectory(tools/)
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

    file(GLOB_RECURSE proc_image_processing_GPU_FILES "${proc_image_processing_GPU_SRC_DIR}/*.cc" "${proc_image_processing_GPU_SRC_DIR}/*.h")
    file(GLOB_RECURSE proc_image_processing_CPU_FILES "${proc_image_processing_CPU_SRC_DIR}/*.cc" "${proc_image_processing_CPU_SRC_DIR}/*.h")

    list(REMOVE_ITEM proc_image_processing_GPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_GPU_SRC_DIR}/main.cc)
    list(REMOVE_ITEM proc_image_processing_CPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_CPU_SRC_DIR}/main.cc)
    list(REMOVE_ITEM proc_image_processing_CPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_CPU_SRC_DIR}/algorithm/major_edge_extractor.cc)
    list(REMOVE_ITEM proc_image_processing_CPU_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_CPU_SRC_DIR}/algorithm/major_edge_extractor.h)

    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${proc_image_processing_SRC_DIR}
            ${sonia_common_INCLUDE_DIRS}
            ${pcl_ros_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )

    add_executable(
            ${PROJECT_NAME}_node
            ${proc_image_processing_GPU_SRC_DIR}/main.cc
            ${proc_image_processing_GPU_FILES}
            ${proc_image_processing_CPU_FILES}
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
endfunction()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs image_transport cv_bridge roslaunch pcl_ros sonia_common)

find_package(CUDA)
if (CUDA_FOUND)
    include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
            ${CMAKE_BINARY_DIR} tools/detect_gpu.c
            CMAKE_FLAGS
            -DINCLUDE_DIRECTORIES=${CUDA_TOOLKIT_INCLUDE}
            -DLINK_LIBRARIES=${CUDA_CUDART_LIBRARY}
            COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
            RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR
            )
    if (COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
        message("##### - Using GPU mode, ${RUN_OUTPUT_VAR} - #####")
        gpu_mode()
    else ()
        message("##### - Using CPU mode, ${RUN_OUTPUT_VAR} - #####")
        cpu_mode()
    endif ()
else ()
    message("##### - Using CPU mode, CUDA not found - #####")
    cpu_mode()
endif (CUDA_FOUND)

add_executable(republisher_node ${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/republisher.cc)
target_link_libraries(republisher_node ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
add_dependencies(republisher_node ${catkin_EXPORTED_TARGETS})