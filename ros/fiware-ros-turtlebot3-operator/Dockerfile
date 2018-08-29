FROM ubuntu:16.04
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ENV PYTHONUNBUFFERED 1

COPY ./kube_entrypoint.sh /opt/kube_entrypoint.sh
WORKDIR /opt/ros_ws

RUN apt update && apt upgrade -y && apt install -y git ca-certificates --no-install-recommends && \
    mkdir -p /opt/ros_ws/src && \
    git clone https://github.com/tech-sketch/fiware_ros_turtlebot3_operator.git src/fiware_ros_turtlebot3_operator && \
    git clone https://github.com/tech-sketch/fiware_ros_turtlebot3_bridge.git src/fiware_ros_turtlebot3_bridge && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get purge -y --auto-remove git
