#!/bin/sh

if [[ -f /mnt/mmcblk0p2/config/disallow_wifi ]]; then
	exit 1
fi

if [[ -f /mnt/mmcblk0p2/config/disallow_ssh ]]; then
	exit 1
fi

if [[ ! -f /usr/local/etc/init.d/openssh ]]; then
	exit 1
fi

SSH_STORE=/mnt/mmcblk0p2/ssh_keys
SSH_DIR=/usr/local/etc/ssh
mkdir -p $SSH_STORE
mkdir -p $SSH_DIR

# Sadly have to generate all keys, else openssh will generate them itself anyway
ALGORITHMS="rsa dsa ecdsa ed25519"

for ALG in $ALGORITHMS; do
	KEY=$SSH_STORE/ssh_host_${ALG}_key

	# Generate SSH keys if not available
	if [[ ! -f $KEY || ! -f $KEY.pub ]]; then
		ssh-keygen -t ${ALG} -N "" -f $KEY
	fi

	# Make keys accessible for OpenSSH (could also link)
	cp $KEY $SSH_DIR
	cp $KEY.pub $SSH_DIR
done

if [[ -f /mnt/mmcblk0p2/config/wireless_autoconnect && -f /mnt/mmcblk0p2/config/enable_ssh ]]; then
	# Start OpenSSH
	/usr/local/etc/init.d/openssh start
fi