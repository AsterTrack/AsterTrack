#!/bin/sh

cd /mnt/mmcblk0p4/kernel/source

echo "Preparing for kernel modules building..."

CONFIG=config-$(uname -r)

make clean
make KCONFIG_ALLCONFIG=../${CONFIG}/.config alldefconfig -j 4 || exit 1
make prepare -j 4 || exit 1
make modules_prepare -j 4 || exit 1

cp -f ../${CONFIG}/System.map .
cp -f ../${CONFIG}/Module.symvers .

sync

echo "Finished preparing for kernel modules!"

cd /home/tc