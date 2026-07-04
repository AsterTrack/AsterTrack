#!/bin/sh

# Ensure setup of swap and storage partitions (requires reboot after first creation)
/opt/part.sh

# Update any data stores
/opt/data.sh

# Setup and connect wifi if configured
/opt/wifi.sh

# Setup and start OpenSSH if configured
/opt/ssh.sh

# Set swappiness very low, SD card swap should not be used unless there's no other way 
sudo sysctl vm.swappiness=1

# Make sure we have a valid ID
IDPATH=/mnt/mmcblk0p4/config/id
if [[ "$(<$IDPATH wc -c)" != 4 ]]; then
	sudo dd if=/dev/urandom of=$IDPATH bs=1 count=4
	sync
fi
while [ "$(cat $IDPATH | od -N 4 -A n -t d4)" == 0 ]; do
	sudo dd if=/dev/urandom of=$IDPATH bs=1 count=4
	sync
done

# Do any compilation required
STORAGE=/mnt/mmcblk0p4
if [[ ! -f "/home/tc/TrackingCamera/TrackingCamera_$(uname -m)" ]]; then
	# Program binary wasn't installed, assume we are equipped to build
	/home/tc/build_release.sh 1> $STORAGE/build.log 2> $STORAGE/build.err
fi
if [[ ! -f "/home/tc/TrackingCamera/qpu_blob_tiled_min.bin" ]]; then
	# QPU program blob wasn't installed, assume we are equipped to build
	/home/tc/build_qpu.sh 1> $STORAGE/build.log 2> $STORAGE/build.err
fi
if [[ ! -d /home/tc/drivers ]]; then
	# No existing camera drivers, try to built (requires internet and build dependencies)
	/home/tc/drivers_auto_build.sh 1> $STORAGE/build_drivers.log 2> $STORAGE/build_drivers.err
fi

# Try to unmount, works only if all loaded TCEs were configured with copy2fs
sudo umount /mnt/mmcblk0p2

# Set performance profile
echo ondemand > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Allow for realtime threads to take 100% of CPU time
echo -1 > /proc/sys/kernel/sched_rt_runtime_us

# Enable I2C driver
/sbin/modprobe i2c-dev

# Load (custom) camera drivers
/home/tc/drivers_load_modules.sh

# Start tracking camera program loop
/home/tc/run.sh
