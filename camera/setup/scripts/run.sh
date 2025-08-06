#!/bin/sh

ARCH=$(uname -m)
ID=$(cat /mnt/mmcblk0p2/id | od -N 4 -A n -t d4)
LOGPATH=/home/tc/trcam.log

while :
do
        echo "===================================" | tee $LOGPATH
        echo "Tracking Program for $ARCH with ID $ID" | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        sudo /home/tc/TrackingCamera/TrackingCamera_$ARCH --program /home/tc/TrackingCamera/qpu_blob_tiled_min.bin -id $ID --nostatlog -u 2>&1 | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        echo "Tracking Program exited, restarting" | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        if [[ -f /mnt/mmcblk0p2/log ]]; then
                [ -f /mnt/mmcblk0p2/trcam.log  ] && sudo mv /mnt/mmcblk0p2/trcam.log /mnt/mmcblk0p2/trcam.log.last
                sudo mv $LOGPATH /mnt/mmcblk0p2/trcam.log
                echo 3 | sudo tee /proc/sys/vm/drop_caches
        fi
        sleep 1s
done