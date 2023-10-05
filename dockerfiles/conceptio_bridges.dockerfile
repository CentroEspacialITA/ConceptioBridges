ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}


ARG WEBSOCKET_PORT=9091
ARG TCP_PORT=9090
ENV WEBSOCKET_PORT_ENV ${WEBSOCKET_PORT}
ENV TCP_PORT_ENV ${TCP_PORT}
 
ARG WORKSPACE=/opt/conceptio

RUN apt update -y && apt dist-upgrade -y && apt install -y python3-pip

# TODO: separate build in another image (copy only share+lib folders) 
WORKDIR ${WORKSPACE}
COPY [".", "."]
ARG DEBIAN_FRONTEND=noninteractive

RUN pip3 install --upgrade setuptools==58.2.0

RUN rosdep install --from-paths . --ignore-src -r -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
	colcon build
	
WORKDIR ${WORKSPACE}
HEALTHCHECK --interval=10s --timeout=4s \
	CMD ./healthcheck.sh 

RUN ["chmod", "+x", "entrypoint.sh"]
RUN ["chmod", "+x", "healthcheck.sh"]
ENTRYPOINT ["./entrypoint.sh"]
CMD ["9090", "9091"]


