#!/bin/sh

ARCH=$(uname -m)
LOGPATH=/home/tc/trcam.log
LOGPERM=/mnt/mmcblk0p2/trcam.log

while :
do
        echo "===================================" | tee $LOGPATH
        echo "Tracking Program for $ARCH" | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        sudo /home/tc/TrackingCamera/TrackingCamera_$ARCH --program /home/tc/TrackingCamera/qpu_blob_tiled_min.bin --nostatlog -u 2>&1 | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        echo "Tracking Program exited, restarting" | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        if [[ -f /mnt/mmcblk0p2/config/log ]]; then
                cp $LOGPATH $LOGPATH.last
                [ -f $LOGPERM ] && sudo mv $LOGPERM $LOGPERM.last
                sudo mv $LOGPATH $LOGPERM
                sync
        fi
        sleep 1s
done
