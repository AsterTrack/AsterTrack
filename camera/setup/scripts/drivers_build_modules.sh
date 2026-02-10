#!/bin/sh

cd /mnt/mmcblk0p4/kernel/module

make -C ../source M=$PWD clean || exit 1
make -C ../source M=$PWD modules -j 2 || exit 1

sync

mkdir -p /home/tc/drivers
cp ov9281.ko /home/tc/drivers/
cp ov9282.ko /home/tc/drivers/

sync

echo "Finished building kernel drivers!"

cd /home/tc