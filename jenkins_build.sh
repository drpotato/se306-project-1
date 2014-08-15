#!/bin/bash

# Required for the Jenkins user to build the project
source /opt/ros/indigo/setup.bash
sudo rosdep init
rosdep update

$WORKSPACE/build.sh
if [ $? -ne 0 ]; then
 echo "Jenkins could not build this!"
 exit 1
fi

# Create a tarball with everything needed to run
mkdir -p se306_runnable/src/se306_p1_pkg

cp run.sh se306_runnable/run.sh
cp -r src/se306_p1_pkg/world/ se306_runnable/src/se306_p1_pkg/world/
cp -r devel/ se306_runnable/devel/

tar -cvf se306_runnable.tar se306_runnable
rm -rf se306_runnable
