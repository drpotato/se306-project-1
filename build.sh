#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

catkin_make --pkg msg_pkg
catkin_make
python src/se306_p1_pkg/world/updateactors.py
