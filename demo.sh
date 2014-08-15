#!/bin/bash
./build.sh
nohup ./run.sh &
sleep 1s
urxvt -hold -e /bin/bash -c "source devel/setup.bash; rostopic echo socialness" &
urxvt -hold -e /bin/bash -c "source devel/setup.bash; rostopic echo entertainedness" &
urxvt -hold -e /bin/bash -c "source devel/setup.bash; rostopic echo interaction" &
