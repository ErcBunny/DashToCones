#!/bin/bash

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1",MODE="0666", GROUP:="dialout",  SYMLINK+="port1"' >/etc/udev/rules.d/port1.rules
echo  'KERNEL=="tty*", ATTRS{devpath}=="1.2",MODE="0666", GROUP:="dialout",  SYMLINK+="port2"' >/etc/udev/rules.d/port2.rules
echo  'KERNEL=="tty*", ATTRS{devpath}=="1.3",MODE="0666", GROUP:="dialout",  SYMLINK+="port3"' >/etc/udev/rules.d/port3.rules
echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4",MODE="0666", GROUP:="dialout",  SYMLINK+="port4"' >/etc/udev/rules.d/port4.rules
#echo  'KERNEL=="tty*", KERNELS=="usb3", MODE="0666", GROUP:="dialout",  SYMLINK+="port5"' >/etc/udev/rules.d/port5.rules

#echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE="0666", GROUP:="dialout",  SYMLINK+="dashgo"' >/etc/udev/rules.d/dashgo1.rules

#echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP:="dialout",  SYMLINK+="dashgo"' >/etc/udev/rules.d/dashgo2.rules

#echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP:="dialout",  SYMLINK+="flashlidar"' >/etc/udev/rules.d/flashlidar1.rules

#echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP:="dialout",  SYMLINK+="flashlidar"' >/etc/udev/rules.d/flashlidar2.rules

service udev reload
sleep 2
service udev restart
