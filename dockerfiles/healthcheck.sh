#!/bin/bash

node_name_tcp = "/rosbridge_tcp"
node_name_websocket = "/rosbridge_websocket"

node_list=$(ros2 node list)

if [[ $node_list == *"$node_name_tcp"* && $node_list == *"$node_name_websocket"* ]]; then
    exit 0
else
    exit 1
fi

