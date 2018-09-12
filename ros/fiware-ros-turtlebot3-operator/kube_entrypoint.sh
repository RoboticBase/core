#!/bin/bash

rm -f /opt/ros_ws/src/fiware_ros_turtlebot3_operator/config/config.yaml
ln -s /etc/fiware_ros_turtlebot3_operator/configmaps/config.yaml /opt/ros_ws/src/fiware_ros_turtlebot3_operator/config/config.yaml

source /opt/ros/kinetic/setup.bash
catkin_make
source /opt/ros_ws/devel/setup.bash
roslaunch fiware_ros_turtlebot3_operator fiware_ros_turtlebot3_operator.launch
