#!/bin/sh

ARCH=$(uname -m)
LOGPATH=/home/tc/trcam.log
STORAGE=/mnt/mmcblk0p4/trcam.log
CONFIG=/mnt/mmcblk0p4/config

while :
do
        echo "===================================" | tee $LOGPATH
        echo "Tracking Program for $ARCH" | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        sudo /home/tc/TrackingCamera/TrackingCamera_$ARCH --nostatlog -u 2>&1 | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        echo "Tracking Program exited, restarting" | tee $LOGPATH
        echo "===================================" | tee $LOGPATH
        if [[ -f $CONFIG/log ]]; then
                cp $LOGPATH $LOGPATH.last
                [ -f $LOGPERM ] && sudo mv $LOGPERM $LOGPERM.last
                sudo mv $LOGPATH $LOGPERM
                sync
        fi
        sleep 1s
done
