#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

catkin_make
python build_updateActors.py
