#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the hub usb connection as /dev/port*"
echo ""

sudo cp hub-usb.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
