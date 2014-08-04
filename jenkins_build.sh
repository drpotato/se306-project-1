#!/bin/bash

# Required for the Jenkins user to build the project
source /opt/ros/indigo/setup.bash
sudo rosdep init
rosdep update

$WORKSPACE/build.sh
