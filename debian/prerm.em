#!/bin/bash

if [ -e /etc/udev/rules.d/81-tplink.rules ]; then
    rm /etc/udev/rules.d/81-tplink.rules
fi

exit 0
