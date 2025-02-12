#!/bin/sh

if [[ -b "/dev/mmcblk0p3" ]]; then
    sudo mkswap /dev/mmcblk0p3
    sudo swapon /dev/mmcblk0p3
else
    echo "No swap partition exists, use swap_part.sh (beware it will reboot!)"
fi
