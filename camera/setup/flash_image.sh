#!/bin/bash

DEVICE_PATH=$1
IMAGE_FILE=$2

if [[ $DEVICE_PATH = *"h"* ]]; then
	echo "Usage: [sudo] flash_image DEVICE_PATH IMAGE_FILE
	DEVICE_PATH: Path to SD, depends on the SD card reader, usually '/dev/mmcblk0' or a '/dev/sdX' device
	IMAGE_FILE: The TrackingCamera image you created with setup_image.sh"
	exit 0
fi

if [[ "$EUID" != 0 ]]; then
    echo "Need sudo to execute!"
    exit 1
fi

if [[ -z "$DEVICE_PATH" || -z "$IMAGE_FILE" ]]; then
	echo "Please specify the device path as the first argument and a valid image file as the second argument!"
	exit 1
fi
if [[ ! -b "$DEVICE_PATH" ]]; then
	echo "Device $DEVICE_PATH does not exist! Make sure to first specify the SD card reader, then the image file!"
	exit 1
fi
if [[ ! -f "$IMAGE_FILE" ]]; then
	echo "Image file does not exist!"
	exit 1
fi

# Unmount device and all partitions
for i in "$DEVICE_PATH?*"; do umount -q $i; done
umount -q $DEVICE_PATH
# Unfortunately, quiet option doesn't work, ignore "not mounted" messages

echo "Flashing image..."

# Flash with sync - may decrease performance (though blocksize is large) but allows for progress reporting
sudo dd if=$IMAGE_FILE of=$DEVICE_PATH bs=4M status=progress oflag=sync
