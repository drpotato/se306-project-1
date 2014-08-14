#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

python build_includeActors.py
catkin_make
python build_updateActors.py