#!/bin/sh

if [[ ! -b "/dev/mmcblk0p3" ]]; then
    echo "Creating swap partition!"
    sudo fdisk /dev/mmcblk0 << EOF
n
p
3
10000

+1G
p
w
EOF
    #if [[ -b "/dev/mmcblk0p3" ]]; then
    # TODO: Detect when creating partition failed
    echo "Rebooting in 5s... Ctrl+C to cancel"
    sleep 5s
    sudo reboot
    sleep 5s
    #else
    #    echo "Failed to create swap partition! Continuing without..."
    #fi
else
    sudo mkswap /dev/mmcblk0p3
    sudo swapon /dev/mmcblk0p3
fi