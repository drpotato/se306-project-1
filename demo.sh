#!/bin/bash
./build.sh
nohup ./run.sh &
sleep 1s
$TERM -hold -e /bin/bash -c "source devel/setup.bash; rostopic echo socialness" &
$TERM -hold -e /bin/bash -c "source devel/setup.bash; rostopic echo entertainedness" &
$TERM -hold -e /bin/bash -c "source devel/setup.bash; rostopic echo interaction" &
