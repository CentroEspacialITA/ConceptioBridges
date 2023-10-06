ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} as builder
	
ARG WEBSOCKET_PORT=9091
ARG TCP_PORT=9090
ARG MQTT_HOST='localhost'
ARG MQTT_PORT=1883

ENV WEBSOCKET_PORT_ENV ${WEBSOCKET_PORT}
ENV TCP_PORT_ENV ${TCP_PORT}
ENV MQTT_HOST ${MQTT_HOST}
ENV MQTT_PORT ${MQTT_PORT}

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
ENTRYPOINT ["./dockerfiles/entrypoint.sh"]
CMD [${WEBSOCKET_PORT_ENV}, ${TCP_PORT_ENV}, ${MQTT_HOST}, ${MQTT_PORT}]

