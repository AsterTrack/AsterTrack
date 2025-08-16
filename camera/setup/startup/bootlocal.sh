#!/bin/sh

# Ensure swap partition exists and is used (requires reboot after first creation)
/opt/swap.sh

# Setup and connect wifi if configured
/opt/wifi.sh &

# Setup and start OpenSSH if configured
/opt/ssh.sh

if [[ -f /usr/local/etc/init.d/avahi ]]; then
	# Start dbus and avahi for zeroconf (broadcasting hostname)
	/usr/local/etc/init.d/dbus start
	/usr/local/etc/init.d/avahi start
	sed -i 's|files dns$|files mdns4_minimal [NOTFOUND=return] dns|' '/etc/nsswitch.conf'
fi

# Set performance profile
echo ondemand > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Allow for realtime threads to take 100% of CPU time
echo -1 > /proc/sys/kernel/sched_rt_runtime_us

# Enable I2C driver
/sbin/modprobe i2c-dev

# Make sure we have a valid ID
IDPATH=/mnt/mmcblk0p2/config/id
if [[ "$(<$IDPATH wc -c)" != 4 ]]; then
	sudo dd if=/dev/urandom of=$IDPATH bs=1 count=4
fi
while [ "$(cat $IDPATH | od -N 4 -A n -t d4)" == 0 ]; do
	sudo dd if=/dev/urandom of=$IDPATH bs=1 count=4
done

if [[ ! -f "/home/tc/TrackingCamera/TrackingCamera_$(uname -m)" ]]; then
	# Program binary wasn't installed, assume we are equipped to build
	/home/tc/build_release.sh 1> /mnt/mmcblk0p2/build.log 2> /mnt/mmcblk0p2/build.err
fi

/home/tc/run.sh
