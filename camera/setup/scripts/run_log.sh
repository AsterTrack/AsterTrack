#!/bin/sh

ARCH=$(uname -m)
ID=$(cat /mnt/mmcblk0p2/id | od -N 4 -A n -t d4)

while :
do
	[ -f /mnt/mmcblk0p2/trcam.log ] && mv /mnt/mmcblk0p2/trcam.log /mnt/mmcblk0p2/trcam.log.last
	echo "===================================" >> trcam.log
	echo "Tracking Program for $ARCH with ID $ID" >> trcam.log
	echo "===================================" >> trcam.log
	TrackingCamera/TrackingCamera_$ARCH --program TrackingCamera/qpu_blob_tiled_min.bin -id $ID -u 2>&1 > /mnt/mmcblk0p2/trcam.log
	echo "===================================" >> trcam.log
	echo "Tracking Program exited, restarting" >> trcam.log
	echo "===================================" >> trcam.log
	echo 3 | sudo tee /proc/sys/vm/drop_caches
	sleep 1s
done
