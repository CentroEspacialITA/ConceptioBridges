ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}


ARG WEBSOCKET_PORT=9091
ARG TCP_PORT=9090
ENV WEBSOCKET_PORT_ENV ${WEBSOCKET_PORT}
ENV TCP_PORT_ENV ${TCP_PORT}
 
ARG WORKSPACE=/opt/conceptio

RUN apt update -y && apt dist-upgrade -y && apt install -y python3-pip

WORKDIR ${WORKSPACE}
COPY ["conceptio_interfaces", "conceptio_interfaces/"]
COPY ["rosbridge_suite", "rosbridge_suite/"]
ARG DEBIAN_FRONTEND=noninteractive

RUN pip3 install --upgrade setuptools==58.2.0

RUN rosdep install --from-paths . --ignore-src -r -y

WORKDIR ${WORKSPACE}/conceptio_interfaces
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build

# We need the forked version of rosbridge-suite (tsender) to include the TCP protocol.
WORKDIR ${WORKSPACE}/rosbridge_suite
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build
	
WORKDIR ${WORKSPACE}
HEALTHCHECK --interval=10s --timeout=4s \
	CMD ./healthcheck.sh 

COPY ["dockerfiles/entrypoint.sh", "entrypoint.sh"]
COPY ["dockerfiles/healthcheck.sh", "healthcheck.sh"]
RUN ["chmod", "+x", "entrypoint.sh"]
RUN ["chmod", "+x", "healthcheck.sh"]
ENTRYPOINT ["./entrypoint.sh"]
CMD ["9090", "9091"]


