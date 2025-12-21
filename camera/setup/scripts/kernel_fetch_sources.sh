#!/bin/sh

mkdir -p /mnt/mmcblk0p4/kernel
cd /mnt/mmcblk0p4/kernel

REPO_URL="http://www.tinycorelinux.net/16.x/armhf"
KERNEL=6.12.25
PREFIX=6.12.25-piCore-v7

echo "Downloading kernel source and config..."
wget -q $REPO_URL/release/src/kernel/rpi-linux-${KERNEL}.tar.xz
wget -q $REPO_URL/release/src/kernel/${PREFIX}_System.map.xz
wget -q $REPO_URL/release/src/kernel/${PREFIX}_Module.symvers.xz
wget -q $REPO_URL/release/src/kernel/${PREFIX}_.config.xz

rm -rf source config

echo "Decompressing and unpacking kernel source..."
mkdir -p source
tar -xf rpi-linux-${KERNEL}.tar.xz -C source

echo "Unpacking config..."
mkdir -p config
unxz -k ${PREFIX}_.config.xz
mv ${PREFIX}_.config config/.config
unxz -k ${PREFIX}_System.map.xz
mv ${PREFIX}_System.map config/System.map
unxz -k ${PREFIX}_Module.symvers.xz
mv ${PREFIX}_Module.symvers config/Module.symvers