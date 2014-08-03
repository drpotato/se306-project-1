#!/bin/bash

source /opt/ros/indigo/setup.bash
rosdep init
$WORKSPACE/build.sh
