#!/bin/bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

source devel/setup.bash
catkin_make run_tests_se306_p1_pkg_gtest