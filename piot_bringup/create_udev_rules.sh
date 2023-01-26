#!/bin/bash

echo "remap the devices serial port(ttyUSBX) to  RPLidar, Wit-Motion IMU"
echo "devices usb connection as /dev/RPLIDAR, /dev/IMU, check it using the command : ls -l /dev|grep -e ttyUSB "
echo "start copy piot.rules to  /etc/udev/rules.d/"
echo "$HOME/colcon_ws/src/PIOT_ROBOT/piot_bringup/piot.rules"
sudo cp $HOME/colcon_ws/src/PIOT_ROBOT/piot_bringup/piot.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
