#!/bin/bash
set -e

# setup ros environment
source $ROS_WS_SETUP
# setup sonia environment
source $SONIA_WS_SETUP
# start ssh server
echo "test" | sudo -S service ssh start
roscore

exec "$@"