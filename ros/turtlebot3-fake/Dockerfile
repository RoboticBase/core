FROM osrf/ros:kinetic-desktop-full
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ENV PYTHONUNBUFFERED 1
ENV TURTLEBOT3_MODEL waffle

COPY ./kube_entrypoint.sh /opt/kube_entrypoint.sh
WORKDIR /opt/ros_ws

ARG TURTLEBOT3_USER
ENV TURTLEBOT3_USER ${TURTLEBOT3_USER:-turtlebot3}
ARG TURTLEBOT3_UID
ENV TURTLEBOT3_UID ${TURTLEBOT3_UID:-1000}

RUN apt update && apt upgrade -y && apt install -y git ca-certificates --no-install-recommends && \
    mkdir -p /opt/ros_ws/src && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3 && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get purge -y --auto-remove git

RUN useradd -m --uid ${TURTLEBOT3_UID} --shell /bin/bash ${TURTLEBOT3_USER} && \
    chown -R ${TURTLEBOT3_USER} /opt/ros_ws
USER ${USER}
