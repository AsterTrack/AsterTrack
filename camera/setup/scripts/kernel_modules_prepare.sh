#!/bin/sh

cd /mnt/mmcblk0p4/kernel/source

make clean
make KCONFIG_ALLCONFIG=../config/.config alldefconfig
make prepare
make modules_prepare

cp -f ../config/System.map .
cp -f ../config/Module.symvers .

cd /home/tc