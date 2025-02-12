#!/bin/sh
# Make sure we have a valid ID
if [ ! "$(</mnt/mmcblk0p2/id wc -c)" == 4 ]; then
    sudo dd if=/dev/urandom of=/mnt/mmcblk0p2/id bs=1 count=4
fi
while [ "$(cat /mnt/mmcblk0p2/id | od -N 4 -A n -t d4)" == 0 ]; do
    sudo dd if=/dev/urandom of=/mnt/mmcblk0p2/id bs=1 count=4
done
#MARKER
/opt/bootlocal.sh &