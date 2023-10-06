#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.sh
. /opt/conceptio/conceptio_bridges/install/setup.sh
sudo ufw disable
ros2 launch conceptio_bridges conceptio_bridges_launch.py \
    websocket_port:=$1 tcp_port:=$2 \
    mqtt_host:=$3 mqtt_port:=$4
