FROM ubuntu:16.04
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ENV PYTHONUNBUFFERED 1

COPY ./kube_entrypoint.sh /opt/kube_entrypoint.sh

WORKDIR /opt/ros_ws

RUN apt update && apt upgrade -y && mkdir -p /opt/ros_ws/src && rm -rf /var/lib/apt/lists/*
