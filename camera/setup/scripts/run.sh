#!/bin/sh

ARCH=$(uname -m)
ID=$(cat /mnt/mmcblk0p2/id | od -N 4 -A n -t d4)

while :
do
	echo "==================================="
	echo "Tracking Program for $ARCH with ID $ID"
	echo "==================================="
	TrackingCamera/TrackingCamera_$ARCH --program TrackingCamera/qpu_blob_tiled_min.bin -id $ID -u
	echo "==================================="
	echo "Tracking Program exited, restarting"
	echo "==================================="
	sleep 1s
done
