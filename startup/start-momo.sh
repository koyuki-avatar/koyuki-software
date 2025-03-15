#!/bin/bash

cd $KOYUKI_SOFTWARE_PATH/momo

./momo \
    --serial /dev/tnt0,115200 \
    --video-device "usb-0000:00:15.0-2.3" \
    --resolution 640x480 \
    --use-sdl ayame \
    --signaling-url wss://ayame-labo.shiguredo.app/signaling \
    --room-id koyuki-avatar@koyuki1 \
    --signaling-key WTQLW9OdRi9ASuQ4eHH0dDpo4JnZ4-NvrSJj-mKa0I4UjgJI
