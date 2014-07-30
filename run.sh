#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

roscore &
ROSCORE_PID=$!
rosrun se306_example R0 &
rosrun se306_example R1 &
rosrun stage_ros stageros se306_example/world/myworld.world

killall R1
killall R2
kill $ROSCORE_PID