#!/bin/sh

CONFIG=/mnt/mmcblk0p4/config

if [[ -f $CONFIG/disallow_wifi ]]; then
	exit 1
fi

if [[ -f $CONFIG/disallow_ssh ]]; then
	exit 1
fi

if [[ ! -f /usr/local/etc/init.d/openssh ]]; then
	exit 1
fi

SSH_STORE=$CONFIG/ssh_keys
SSH_DIR=/usr/local/etc/ssh
mkdir -p $SSH_STORE
mkdir -p $SSH_DIR

# Sadly have to generate all keys, else openssh will generate them itself anyway
ALGORITHMS="rsa ecdsa ed25519"

for ALG in $ALGORITHMS; do
	KEY=$SSH_STORE/ssh_host_${ALG}_key

	# Generate SSH keys if not available
	if [[ ! -f $KEY || ! -f $KEY.pub ]]; then
		ssh-keygen -t ${ALG} -N "" -f $KEY
		sync
	fi

	# Make keys accessible for OpenSSH (could also link)
	cp $KEY $SSH_DIR
	cp $KEY.pub $SSH_DIR
done

if [[ -f $CONFIG/wireless_autoconnect && -f $CONFIG/enable_ssh ]]; then
	# Start OpenSSH
	/usr/local/etc/init.d/openssh start
fi