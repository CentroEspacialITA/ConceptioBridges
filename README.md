# conceptio_bridges 
![MainBranchConceptioBridges](https://github.com/ConceptioLab/conceptio_bridges/actions/workflows/docker-image.yml/badge.svg?branch=main)

Conceptio_bridges is a ROS2 package that provides some connectivity utilities for the Concept.IO Arena, such as automatic mirroring of MQTT messages to ROS2 messages.

## Usage

There are two ways to use this package: by using its Docker image or by cloning the repository and running it. 


### Using the Docker image

1. Pull the image:
     `docker pull ghcr.io/conceptiolab/conceptio_bridges:main`

2. Run the image with the environment variables MQTT_HOST_ENV and MQTT_PORT_ENV:
   
    `docker run -ti ghcr.io/conceptiolab/conceptio_bridges:main -e MQTT_HOST_ENV="IP_ADDRESS" -e MQTT_PORT_ENV="1883"`


### Cloning the repository

1. Clone the repository:
   
   `git clone https://github.com/ConceptioLab/conceptio_bridges.git`

3. Compile and source the interfaces:
   
   `cd conceptio_interfaces && colcon build && source install/setup.bash`

5. Compile and source the mqtt_mirror package:
   
   `cd mqtt_mirror && colcon build && source install/setup.bash`

7. Compile and source the conceptio_bridges package:
   
   `cd conceptio_bridges && colcon build && source install/setup.bash`

9. Run the package launch file:
    
   `ros2 launch conceptio_bridges conceptio_bridges_launch.py mqtt_host:="IPADDRESS" mqtt_port:=1883`

You could, of course, use only the mqtt_mirror package without the conceptio_bridges launch file, but in the future the launch file will provide more functionality. 
