#!/bin/bash

echo "delete remap the devices serial port(ttyUSBX) to  RPLidar, Wit-Motion IMU"
echo "sudo rm   /etc/udev/rules.d/piot.rules"
sudo rm   /etc/udev/rules.d/piot.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish  delete"
