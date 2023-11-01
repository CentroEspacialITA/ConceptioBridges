ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} as builder
	
ARG MQTT_HOST='emqx'
ARG MQTT_PORT=1883

ENV MQTT_HOST_ENV ${MQTT_HOST}
ENV MQTT_PORT_ENV ${MQTT_PORT}

ARG WORKSPACE=/opt/conceptio

RUN apt update -y && apt dist-upgrade -y && apt install -y python3-pip ufw


WORKDIR ${WORKSPACE}
COPY [".", "${WORKSPACE}/conceptio_bridges/"]
ARG DEBIAN_FRONTEND=noninteractive


WORKDIR ${WORKSPACE}/conceptio_bridges
RUN pip3 install --upgrade setuptools==58.2.0 paho-mqtt

RUN rosdep install --from-paths . --ignore-src -r -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build
	
HEALTHCHECK --interval=10s --timeout=4s \
	CMD ./healthcheck.sh 

RUN ["chmod", "+x", "dockerfiles/entrypoint.sh"]
RUN ["chmod", "+x", "dockerfiles/healthcheck.sh"]
ENTRYPOINT ./dockerfiles/entrypoint.sh ${MQTT_HOST_ENV} ${MQTT_PORT_ENV}


