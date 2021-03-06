#!/bin/bash

mkdir -p /etc/udev/rules.d

cat << EOF > /etc/udev/rules.d/81-tplink.rules

KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2357", ATTRS{idProduct}=="011e", TAG+="uaccess", TAG+="systemd", ENV{SYSTEMD_WANTS}="mocap_pose.service"

SUBSYSTEM=="usb", ATTRS{idVendor}=="2357", ATTRS{idProduct}=="011e", MODE="0666", GROUP="plugdev"
EOF

chmod 644 /etc/udev/rules.d/81-tplink.rules

exit 0
