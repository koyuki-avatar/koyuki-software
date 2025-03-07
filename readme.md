# koyuki-ros-ws
```
colcon build
. /opt/ros/jazzy/setup.bash
. install/setup.bash
ros2 run blv_control blv_control
ros2 run webrtc_datachannel webrtc_datachannel
```

```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
ポートを指定するのが課題。
```
./momo \
--serial /dev/pts/4,115200 \
--video-device "usb-0000:00:15.0-7" \
--resolution 640x480 \
--use-sdl ayame \
--signaling-url wss://ayame-labo.shiguredo.app/signaling \
--room-id koyuki-avatar@<id> \
--signaling-key <replace to key>
```
