#!/bin/sh

if [[ ! -f /mnt/mmcblk0p4/kernel/source/complete_unpack ]]; then
    /home/tc/kernel_fetch_sources.sh
    touch /mnt/mmcblk0p4/kernel/config/complete_unpack
fi

if [[ ! -d /mnt/mmcblk0p4/kernel/module ]]; then
    /home/tc/drivers_fetch_sources.sh
fi

if [[ ! -d /mnt/mmcblk0p4/kernel/source/System.map ]]; then
    /home/tc/kernel_modules_prepare.sh
fi

/home/tc/drivers_build_modules.sh