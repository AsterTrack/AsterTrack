#!/bin/sh

cd /mnt/mmcblk0p4/kernel/source

echo "Preparing for kernel modules building..."

make clean
make KCONFIG_ALLCONFIG=../config/.config alldefconfig -j 4 || exit 1
make prepare -j 4 || exit 1
make modules_prepare -j 4 || exit 1

cp -f ../config/System.map .
cp -f ../config/Module.symvers .

sync

echo "Finished preparing for kernel modules!"

cd /home/tc