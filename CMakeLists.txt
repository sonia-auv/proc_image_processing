     # \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N


cmake_minimum_required(VERSION 2.8.3)

project(proc_image_processing)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s -Wno-unknown-pragmas")

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

set(proc_image_processing_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    image_transport
    cv_bridge
    roslaunch
    pcl_ros
    sonia_common
    )

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

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE proc_image_processing_FILES
    "${proc_image_processing_SRC_DIR}/*.cc"
    "${proc_image_processing_SRC_DIR}/*.h")

list(REMOVE_ITEM proc_image_processing_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/main.cc)
list(REMOVE_ITEM proc_image_processing_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${proc_image_processing_SRC_DIR}/${PROJECT_NAME}/uncompress/republisher.cc)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${proc_image_processing_SRC_DIR}
    ${sonia_common_INCLUDE_DIRS}
    ${pcl_ros_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
#===============================================================================
# C R E A T E   E X E C U T A B L E

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


#============================================================================
# U N I T   T E S T S

# Uncomment this line to compile UT.
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
