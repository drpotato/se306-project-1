#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

roscore &
ROSCORE_PID=$!
rosrun se306_example R0 &
ROBOT1_PID=$!
rosrun se306_example R1 &
ROBOT2_PID=$!
rosrun stage_ros stageros se306_example/world/myworld.world

kill $ROBOT1_PID
kill $ROBOT2_PID
kill $ROSCORE_PID