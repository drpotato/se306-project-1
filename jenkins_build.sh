#!/bin/bash

# Required for the Jenkins user to build the project
source /opt/ros/indigo/setup.bash
sudo rosdep init
rosdep update

$WORKSPACE/build.sh

# Create a tarball with everything needed to run
mkdir -p s$WORKSPACE/e306_runnable/src/se306_p1_pkg

cp $WORKSPACE/run.sh $WORKSPACE/se306_runnable/run.sh
cp -r $WORKSPACE/src/se306_p1_pkg/world/ $WORKSPACE/se306_runnable/src/se306_p1_pkg/world/
cp -r $WORKSPACE/devel/ $WORKSPACE/se306_runnable/devel/

tar -cvf $WORKSPACE/se306_runnable.tar $WORKSPACE/se306_runnable
rm -rf $WORKSPACE/se306_runnable
