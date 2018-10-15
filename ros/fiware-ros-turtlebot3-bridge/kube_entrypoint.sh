#!/bin/bash

PIDS=""
trap 'kill -TERM $PIDS' TERM INT

ln -s /etc/fiware_ros_turtlebot3_bridge/certs/ca.crt /opt/ros_ws/src/fiware_ros_turtlebot3_bridge/secrets/ca.crt
ln -s /etc/fiware_ros_turtlebot3_bridge/secrets/mqtt.yaml /opt/ros_ws/src/fiware_ros_turtlebot3_bridge/config/mqtt.yaml
rm -f /opt/ros_ws/src/fiware_ros_turtlebot3_bridge/config/turtlebot3_cmd.yaml
ln -s /etc/fiware_ros_turtlebot3_bridge/configmaps/turtlebot3_cmd.yaml /opt/ros_ws/src/fiware_ros_turtlebot3_bridge/config/turtlebot3_cmd.yaml
rm -f /opt/ros_ws/src/fiware_ros_turtlebot3_bridge/config/turtlebot3_attrs.yaml
ln -s /etc/fiware_ros_turtlebot3_bridge/configmaps/turtlebot3_attrs.yaml /opt/ros_ws/src/fiware_ros_turtlebot3_bridge/config/turtlebot3_attrs.yaml

source /opt/ros/kinetic/setup.bash
catkin_make
source /opt/ros_ws/devel/setup.bash
roslaunch fiware_ros_turtlebot3_bridge fiware_ros_turtlebot3_bridge.launch &
PIDS="$PIDS $!"

wait $PIDS
trap - TERM INT
wait $PIDS
