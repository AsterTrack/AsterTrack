#!/bin/sh

cd /mnt/mmcblk0p4/kernel/module

make -C ../source M=$PWD clean
make -C ../source M=$PWD modules

mkdir -p /home/tc/drivers
cp ov9281.ko /home/tc/drivers/
cp ov9282.ko /home/tc/drivers/

cd /home/tc