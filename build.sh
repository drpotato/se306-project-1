#!/bin/bash
source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

rosmake se306_example
