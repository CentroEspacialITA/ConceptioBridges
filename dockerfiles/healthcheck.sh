#!/bin/bash

node_name_mqtt_mirror = "/mqtt_mirror_node"

node_list=$(ros2 node list)

if [[ $node_list == *"$node_name_mqtt_mirror"*]]; then
    exit 0
else
    exit 1
fi

