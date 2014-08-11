#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

roscore &
ROSCORE_PID=$!

rosrun map_server map_server se306_p1_package/world/myworld.yaml &
WORLD_PID=$!

rosrun fake_localization fake_localization -odom_frame_id robot_0/odom -base_frame_id robot_0/base_link &
ROSFAKE_PID=$!
rosrun se306_p1_package R0 &
ROBOT1_PID=$!
#rosrun se306_p1_package R1 &
rosrun move_base move_base  &
ROBOT2_PID=$!


rosrun stage_ros stageros se306_p1_package/world/myworld.world
kill $WORLD_PID
kill $ROBOT1_PID
kill $ROBOT2_PID
kill $ROSFAKE_PID
kill $ROSCORE_PID
