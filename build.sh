#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

rosmake se306_p1_package
python se306_p1_package/world/updateactors.py