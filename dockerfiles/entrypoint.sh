#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.sh
. /opt/conceptio/conceptio_msgs/install/setup.sh
. /opt/conceptio/rosbridge_suite/install/setup.sh
ufw disable
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$1 &
ros2 launch rosbridge_server rosbridge_tcp_launch.xml port:=$2 