#!/bin/bash

source /opt/ros/kinetic/setup.bash
catkin_make
source /opt/ros_ws/devel/setup.bash
roslaunch turtlebot3_fake turtlebot3_fake.launch
