#!/bin/sh

if [[ ! -b "/dev/mmcblk0p3" || ! -b "/dev/mmcblk0p4" ]]; then
    # Delete any existing partition 3 or 4, 3 is transient, 4 will be re-created at the exact place
    # Create partition 3 (swap) directly after 2 (TCE), until sector 4194303, so it should be 1-1.5GB
    # Create partition 4 (storage) at 4194304 until end, predictable position allows for recovery of old data
    # NOTE: new primary 4 is NOT asked for number, since maximum is 4, but that might be busybox specific, or might change
    #       and it doesn't hurt to set 4 since next prompt has minimum of 16 anyway
    sudo fdisk /dev/mmcblk0 << EOF
d
3
d
4
n
p
3
10000

4194303
n
p
4
4194304


p
w
p
w
EOF

    # TODO: Detect when creating partition failed
    echo "Rebooting in 5s... Ctrl+C to cancel"
    sleep 5s
    sudo reboot
    sleep 5s
fi

# Setup swap partition
sudo mkswap /dev/mmcblk0p3
sudo swapon /dev/mmcblk0p3

# Ensure storage partition is initialised as ext4
blkid | grep "/dev/mmcblk0p4" | grep "storage" | grep "ext4" >> /dev/null 2>&1
if [[ $? == 1 ]]; then
    echo "Formatting storage"
    sudo mkfs.ext4 -F -L storage /dev/mmcblk0p4
    sudo rebuildfstab
fi

# Mount storage partition on boot according to fstab
sudo mkdir -p /mnt/mmcblk0p4
sudo mount /mnt/mmcblk0p4