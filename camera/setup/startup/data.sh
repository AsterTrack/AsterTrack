#!/bin/sh

CONFIG=/mnt/mmcblk0p4/config
if [[ ! -d $CONFIG ]]; then
    # Copy default config (or migrate)
    sudo cp -r /mnt/mmcblk0p2/config $CONFIG
    if [[ -d /mnt/mmcblk0p2/ssh_keys ]]; then
        # Migrate existing keys from old location
        sudo mv /mnt/mmcblk0p2/ssh_keys $CONFIG/
    fi
fi

# Copy camera MCU firmwware into RAM so TCE partition can be unmounted
cp /mnt/mmcblk0p2/tce/TrackingCameraMCU.bin /tmp/TrackingCameraMCU.bin