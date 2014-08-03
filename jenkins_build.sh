#!/bin/bash

source /opt/ros/indigo/setup.bash
sudo rosdep init
rosdep update
$WORKSPACE/build.sh
