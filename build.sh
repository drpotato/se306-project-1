#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

python build_updateBuild.py
catkin_make
if [ $? -ne 0 ]; then
 echo "The build failed"
 exit 1
fi

python build_updateRun.py

chmod 755 ./run.sh
