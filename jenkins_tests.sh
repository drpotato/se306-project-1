#!/bin/bash

# Required for the Jenkins user to build the project
source /opt/ros/indigo/setup.bash
sudo rosdep init
rosdep update

$WORKSPACE/tests.sh
if [ $? -ne 0 ]; then
 echo "Jenkins could run the tests!"
 exit 1
fi
