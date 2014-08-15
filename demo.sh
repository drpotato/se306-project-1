#!/bin/bash
./build.sh
nohup ./run.sh &
sleep 3s
$TERM -hold -title "socialness" -e $SHELL -c "source devel/setup.bash; rostopic echo socialness" &
$TERM -hold -title "entertainedness" -e $SHELL -c "source devel/setup.bash; rostopic echo entertainedness" &
$TERM -hold -title "interaction" -e $SHELL -c "source devel/setup.bash; rostopic echo interaction" &
