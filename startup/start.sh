#!/bin/bash

cd $KOYUKI_SOFTWARE_PATH

. /opt/ros/jazzy/setup.bash
. ros-ws/install/setup.bash

gnome-terminal --tab -- bash -c "ros2 run blv_control blv_control; exec bash"
gnome-terminal --tab -- bash -c "ros2 launch p9n_bringup teleop.launch.py linear_speed:=0.86 angular_speed:=2.0; exec bash"
gnome-terminal --tab -- bash -c "ros2 run webrtc_datachannel webrtc_datachannel --ros-args -p serial_port:=/dev/tnt1; exec bash"

gnome-terminal --tab -- bash -c " \
./momo/momo \
    --serial /dev/tnt0,115200 \
    --video-device "usb-0000:00:15.0-2.2" \
    --resolution 640x480 \
    --use-sdl ayame \
    --signaling-url wss://ayame-labo.shiguredo.app/signaling \
    --room-id koyuki-avatar@koyuki1 \
    --signaling-key WTQLW9OdRi9ASuQ4eHH0dDpo4JnZ4-NvrSJj-mKa0I4UjgJI \
; exec bash
"
