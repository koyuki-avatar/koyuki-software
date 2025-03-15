#!/bin/bash

# before run this, do:
# install tty0tty and make it persisting across boot by running:
# echo "tty0tty" | sudo tee /etc/modules-load.d/tty0tty.conf

cd $KOYUKI_SOFTWARE_PATH

. /opt/ros/jazzy/setup.bash
. install/setup.bash

ros2 run webrtc_datachannel webrtc_datachannel --ros-args -p serial_port:=/dev/tnt1
