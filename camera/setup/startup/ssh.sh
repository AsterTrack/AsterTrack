#!/bin/sh

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

	# Make keys accessible for OpenSSH (could also link)
	if [[ -f $KEY && -f $KEY.pub ]]; then
		cp $KEY $SSH_DIR
		cp $KEY.pub $SSH_DIR
	fi
done

# Start OpenSSH to generate keys (blocking)
/usr/local/etc/init.d/openssh start

# Ensure the stored keys are the same
for ALG in $ALGORITHMS; do
	KEY=ssh_host_${ALG}_key
	# Only write to SD if keys changed (or don't exist)
	if !cmp --silent -- "$SSH_DIR/$KEY" "$SSH_STORE/$KEY"; then
		cp $SSH_DIR/$KEY $SSH_STORE
	fi
	if ! cmp --silent -- "$SSH_DIR/$KEY.pub" "$SSH_STORE/$KEY.pub"; then
		cp $SSH_DIR/$KEY.pub $SSH_STORE
	fi
done