#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
. devel/setup.bash

rosrun upstage_pkg Upstage