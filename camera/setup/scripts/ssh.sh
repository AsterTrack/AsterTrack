#!/bin/sh

STORE=/mnt/mmcblk0p2/ssh_keys
SSHDIR=/usr/local/etc/ssh
mkdir -p $STORE

# Have to generate/backup all key types, else openssh will generate them anyway
ALGORITHMS="rsa ecdsa ed25519"
MISSING="false"

# Restore already generated keys from backup
for ALG in $ALGORITHMS; do
	KEY=$STORE/ssh_host_${ALG}_key

	if [[ -f $KEY && -f $KEY.pub ]]; then
		cp $KEY $SSHDIR
		cp $KEY.pub $SSHDIR
	else
		MISSING="true"
	fi
done

if [[ "$MISSING" == "true" ]]; then

	# Start OpenSSH to generate keys (blocking), and then immediately stop
	/usr/local/etc/init.d/openssh start
	/usr/local/etc/init.d/openssh stop

	# Backup newly generated keys
	for ALG in $ALGORITHMS; do
		KEY=$SSHDIR/ssh_host_${ALG}_key

		cp $KEY $STORE
		cp $KEY.pub $STORE
	done
fi
