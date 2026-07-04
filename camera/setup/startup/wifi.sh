#!/bin/sh

CONFIG=/mnt/mmcblk0p4/config

if [[ -f $CONFIG/disallow_wifi ]]; then
	exit 1
fi

if [[ ! -f /usr/local/sbin/wpa_supplicant ]]; then
	exit 1
fi

if [[ -f $CONFIG/wpa_supplicant.conf ]]; then
	cp $CONFIG/wpa_supplicant.conf /etc/wpa_supplicant.conf
fi

if [[ -f $CONFIG/wireless_autoconnect ]]; then
	# Start wpa_supplicant which will keep us connected to the best network
	wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf -B
	udhcpc -n -i wlan0 -x hostname:$(hostname -s) -F $(hostname -s) &
fi

ntpd --help >> /dev/null 2>&1
if [[ $? == 0 ]]; then
	# Start ntp daemon to ensure timesync (even if we connect only later)
	ntpd --panicgate pool.ntp.org &
fi

if [[ -f /usr/local/etc/init.d/avahi ]]; then
	# Start dbus and avahi for zeroconf (broadcasting hostname)
	/usr/local/etc/init.d/dbus start
	/usr/local/etc/init.d/avahi start
	sed -i 's|files dns$|files mdns4_minimal [NOTFOUND=return] dns|' '/etc/nsswitch.conf'
fi