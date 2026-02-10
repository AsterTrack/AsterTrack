#!/bin/sh

sudo mkdir -p /mnt/mmcblk0p4/kernel
sudo chown -R ${USER} /mnt/mmcblk0p4/kernel
cd /mnt/mmcblk0p4/kernel

REPO_URL="http://www.tinycorelinux.net/16.x/armhf"
KERNEL=6.12.25
PREFIX=6.12.25-piCore-v7

echo "Downloading kernel source and config..."
if [[ ! -f "rpi-linux-${KERNEL}.tar.xz" ]]; then
    wget -q "$REPO_URL/release/src/kernel/rpi-linux-${KERNEL}.tar.xz" || exit 1
fi
if [[ ! -f "${PREFIX}_System.map.xz" ]]; then
    wget -q "$REPO_URL/release/src/kernel/${PREFIX}_System.map.xz" || exit 1
fi
if [[ ! -f "${PREFIX}_Module.symvers.xz" ]]; then
    wget -q "$REPO_URL/release/src/kernel/${PREFIX}_Module.symvers.xz" || exit 1
fi
if [[ ! -f "${PREFIX}_.config.xz" ]]; then
    wget -q "$REPO_URL/release/src/kernel/${PREFIX}_.config.xz" || exit 1
fi

sync

rm -rf source config

echo "Decompressing and unpacking kernel source..."
mkdir -p source
tar -xf "rpi-linux-${KERNEL}.tar.xz" -C source || { echo "rpi-linux source failed to unpack!"; exit 1; }

echo "Unpacking config..."
mkdir -p config
unxz -k "${PREFIX}_.config.xz" || { echo "_.config failed to unpack!"; exit 1; }
mv "${PREFIX}_.config" "config/.config"
unxz -k "${PREFIX}_System.map.xz" || { echo "_System.map failed to unpack!"; exit 1; }
mv "${PREFIX}_System.map" "config/System.map"
unxz -k "${PREFIX}_Module.symvers.xz" || { echo "_Module.symvers failed to unpack!"; exit 1; }
mv "${PREFIX}_Module.symvers" "config/Module.symvers"

sync

echo "Finished fetching kernel sources!"
touch /mnt/mmcblk0p4/kernel/source/complete_unpack

sync