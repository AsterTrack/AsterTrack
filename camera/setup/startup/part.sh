#!/bin/sh

if [[ ! -b "/dev/mmcblk0p3" || ! -b "/dev/mmcblk0p4" ]]; then
    # Delete any existing partition 3 and 4, 3 is transient, 4 will be re-created at a known place without data loss
    # Create partition 3 (swap) from sector 1048576 to 4194303, so it should be 1.5GB
    # Create partition 4 (storage) at 4194304 until end, predictable position allows for recovery of existing data
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
1048576
4194303
n
p
4
4194304

p
w
EOF

    # Similarly, delete and recreate partition 2 (tce) directly after 1 (core), until sector 1048575, so core+TCE should be 500MB
    # This is separate as deleting 2 first is dangerous since fdisk is not designed for scripting
    # If 2 is deleted, and 3 and 4 doesn't exist yet, trying to delete them will just delete partition 1...
    sudo fdisk /dev/mmcblk0 << EOF
d
2
n
p
2
10000

1048575
p
w
EOF

    # TODO: Detect when creating partition failed
    echo "Rebooting in 5s... Ctrl+C to cancel"
    sleep 5s
    sudo reboot
    sleep 5s
fi

# Ensure TCE partition is resized to new extends. Can't do it right after resizing partition, can easily run it every time
sudo resize2fs /dev/mmcblk0p2

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