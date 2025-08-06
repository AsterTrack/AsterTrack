#!/bin/sh

# Ensure swap partition exists and is used (requires reboot after first creation)
/opt/swap.sh

if [[ -f /mnt/mmcblk0p2/wifi.db ]]; then
	# Autoconnect to best known wifi
	/opt/wifi.sh -b &
	# Ensure timesync
	#ntpdate -b pool.ntp.org # This only runs once
	ntpd --panicgate pool.ntp.org # Keeps daemon alive
fi

if [[ -f /usr/local/etc/init.d/openssh ]]; then
	# Setup and start OpenSSH
	/opt/ssh.sh
fi

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
if [[ "$(</mnt/mmcblk0p2/id wc -c)" != 4 ]]; then
	sudo dd if=/dev/urandom of=/mnt/mmcblk0p2/id bs=1 count=4
fi
while [ "$(cat /mnt/mmcblk0p2/id | od -N 4 -A n -t d4)" == 0 ]; do
	sudo dd if=/dev/urandom of=/mnt/mmcblk0p2/id bs=1 count=4
done

if [[ ! -f "/home/tc/TrackingCamera/TrackingCamera_$(uname -m)" ]]; then
	# Program binary wasn't installed, assume we are equipped to build
	/home/tc/build_release.sh 1> /mnt/mmcblk0p2/build.log 2> /mnt/mmcblk0p2/build.err
fi

cd /home/tc
/home/tc/run.sh
