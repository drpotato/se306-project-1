#!/bin/bash
# This file is automatically generated by build_updateActors.py
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

source devel/setup.bash
roscore & ROSCORE_PID=$!
rosrun se306_p1_pkg ActorSpawner 0 R0 -5 3 0 & ROBOT0_PID=$!
rosrun se306_p1_pkg ActorSpawner 1 R1 -6 -3 0 & ROBOT1_PID=$!
rosrun se306_p1_pkg ActorSpawner 2 Resident -3 3 0 & ROBOT2_PID=$!
rosrun se306_p1_pkg ActorSpawner 3 EntertainmentRobot 6 3 0 & ROBOT3_PID=$!

rosrun stage_ros stageros src/se306_p1_pkg/world/myworld.world

kill $ROBOT0_PID
kill $ROBOT1_PID
kill $ROBOT2_PID
kill $ROBOT3_PID
kill $ROSCORE_PID