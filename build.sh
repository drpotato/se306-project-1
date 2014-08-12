#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

catkin_make se306_p1_pkg
python src/se306_p1_pkg/world/updateactors.py
