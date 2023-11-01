#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.sh
. /opt/conceptio/conceptio_bridges/install/setup.sh
ros2 launch conceptio_bridges conceptio_bridges_launch.py \
     mqtt_host:=$1 mqtt_port:=$2
