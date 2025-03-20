# Startup Scripts
**Before running these scripts** do:

- export path

    ```bash
    echo "export KOYUKI_SOFTWARE_PATH=/home/koyuki/koyuki-software" >> ~/.bashrc
    source ~/.bashrc
    ```
- install tty0tty

    install tty0tty and make it persisting across boot by running:

    ```bash
    echo "tty0tty" | sudo tee /etc/modules-load.d/tty0tty.conf
    ```
- chmod

    ```bash
    chmod +x *.sh
    ```
    
- ros2 related:

```
sudo apt install joystick
sudo apt install ros-jazzy-joyz*
```

- momo related:

```
sudo apt install libva2 libva-drm
```
