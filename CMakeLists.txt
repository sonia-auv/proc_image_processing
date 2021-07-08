cmake_minimum_required(VERSION 3.10)
project(proc_image_processing)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -Wall -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -s -Wno-unknown-pragmas")

set(proc_image_processing_SRC_DIR "src")
set(proc_image_processing_IMPL_DIR "src/proc_image_processing")


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs image_transport cv_bridge roslaunch pcl_ros sonia_common)

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

file(GLOB_RECURSE proc_image_processing_FILES "${proc_image_processing_IMPL_DIR}/*.cc" "${proc_image_processing_IMPL_DIR}/*.h")

list(REMOVE_ITEM proc_image_processing_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_IMPL_DIR}/uncompress/republisher.cc)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_image_processing_SRC_DIR}
        ${sonia_common_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}_node
        ${proc_image_processing_IMPL_DIR}/main.cc
        ${proc_image_processing_FILES}
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

add_executable(republisher_node ${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/uncompress/republisher.cc)
target_link_libraries(republisher_node ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
add_dependencies(republisher_node ${catkin_EXPORTED_TARGETS})