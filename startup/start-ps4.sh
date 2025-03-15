#!/bin/bash

cd $KOYUKI_SOFTWARE_PATH

. /opt/ros/jazzy/setup.bash
. install/setup.bash

ros2 launch p9n_bringup teleop.launch.py linear_speed:=0.86 angular_speed:=2.0
