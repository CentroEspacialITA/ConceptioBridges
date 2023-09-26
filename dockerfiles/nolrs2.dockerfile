# Dockerfile for WARA-PS ROS2 arena

# UNCOMMENT FOR HUMBLE 
ARG ROS_DISTRO=humble

ARG WORKSPACE=/opt/conceptio

FROM ros:${ROS_DISTRO} as update_stage

# VNC/X11 Server + camera
RUN apt update -y && apt dist-upgrade -y && apt install -y python3-pip xvfb x11vnc fluxbox guvcview fswebcam ffmpeg \   
	# Remote-ID 
	bluez libgps-dev libconfig-dev libbluetooth-dev nano wget gpsd \
	paho-mqtt xacro pyproj 


FROM update_stage as dependency_stage

WORKDIR /opt/
COPY ["lrs2", "lrs2/"]
COPY ["conceptio", "conceptio/"]
COPY ["aws", "aws/"]

ARG DEBIAN_FRONTEND=noninteractive

RUN pip3 install --upgrade setuptools==58.2.0

RUN rosdep install --from-paths lrs2/ --ignore-src -r -y

RUN rosdep install --from-paths conceptio/ --ignore-src -r -y


FROM dependency_stage as compilation_stage

#RUN apt install -y ros-humble-xacro ros-humble-turtlesim \ 
#	ros-humble-geographic-msgs ros-humble-gazebo-msgs \
#	 python3-colcon-ros python3-colcon-bash python3-pyproj

#RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
#	apt-get update && rosdep install --from-paths \
#	lrs2/ --ignore-src -y -r 

WORKDIR /opt/conceptio/conceptio_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build

# We need the forked version of rosbridge-suite (tsender) to include the TCP protocol.
#RUN apt-get install -y ros-${ROS_DISTRO}-rosbridge-suite
WORKDIR /opt/conceptio/rosbridge_suite
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build

WORKDIR /opt/conceptio/remote_id/RemoteID
RUN mkdir build && \
	cd build && \
 	cmake .. && \
  	make

WORKDIR /opt/conceptio/remote_id/ros2_ws
RUN colcon build

RUN apt install python3-rosinstall -y

WORKDIR /opt/aws/aws-deepracer-interfaces-pkg/deepracer_interfaces_pkg
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build


WORKDIR /opt
COPY ["docker/ros_entrypoint.sh", "/opt/ros_entrypoint.sh"]
RUN ["chmod", "+x", "/opt/ros_entrypoint.sh"]

COPY ["docker/.bashrc", "/root/.bashrc"]

CMD /bin/bash -c "/opt/ros_entrypoint.sh && bash"

