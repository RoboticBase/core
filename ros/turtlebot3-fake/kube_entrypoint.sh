#!/bin/bash

PIDS=""
trap 'kill -TERM $PIDS' TERM INT

source /opt/ros/kinetic/setup.bash
catkin_make
source /opt/ros_ws/devel/setup.bash
roslaunch turtlebot3_fake turtlebot3_fake.launch &
PIDS="$PIDS $!"

wait $PIDS
trap - TERM INT
wait $PIDS
