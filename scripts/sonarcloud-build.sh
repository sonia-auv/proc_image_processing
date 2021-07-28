#!/bin/bash
source "${ROS_WS_SETUP}"
source "${BASE_LIB_WS_SETUP}"
cmake -GNinja .
ninja
ninja tests