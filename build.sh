#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

python build_includeActors.py
catkin_make
if [ $? -ne 0 ]; then
 echo "The build failed"
 exit 1
fi

python build_updateActors.py
