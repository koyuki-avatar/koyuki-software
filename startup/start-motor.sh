#!/bin/bash

cd $KOYUKI_SOFTWARE_PATH

. /opt/ros/jazzy/setup.bash
. install/setup.bash

ros2 run blv_control blv_control
