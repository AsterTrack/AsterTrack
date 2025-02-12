#!/bin/sh

START=$1

STORE=/mnt/mmcblk0p2/ssh_keys
SSHDIR=/usr/local/etc/ssh
mkdir -p $STORE

# This still doesn't work as intended, cannot generate RSA SSH keys without crashing kernel at boot

# Sadly have to generate all keys, else openssh will generate them itself anyway
ALGORITHMS="rsa dsa ecdsa ed25519"
#for ALG in $ALGORITHMS; do
#	KEY=$STORE/ssh_host_${ALG}_key
#
#	if [[ ! -f $KEY || ! -f $KEY.pub ]]; then
#		# Create key in permanent storage
#		rm $KEY
#		rm $KEY.pub
#		if [[ $ALG == "rsa" ]]; then
#			ssh-keygen -b 2048 -t $ALG -f $KEY -N "test"
#			# For some reason, rsa keys generated this way crash the kernel on next boot, same error+line as here:
#			# https://github.com/raspberrypi/linux/issues/3769
#		else
#			ssh-keygen -t $ALG -f $KEY -N ""
#		fi
#	fi
#
#	# Make keys accessible for OpenSSH (could also link)
#	cp $KEY $SSHDIR
#	cp $KEY.pub $SSHDIR
#done

for ALG in $ALGORITHMS; do
	KEY=$STORE/ssh_host_${ALG}_key

	# Make keys accessible for OpenSSH (could also link)
	if [[ -f $KEY && -f $KEY.pub ]]; then
		cp $KEY $SSHDIR
		cp $KEY.pub $SSHDIR
	fi
done

if [[ "$START" == "start" ]]; then
	# If desired, start OpenSSH
	/usr/local/etc/init.d/openssh start
else
	#for ALG in $ALGORITHMS; do
	#	KEY=$STORE/ssh_host_${ALG}_key
	#	if [[ ! -f $KEY || ! -f $KEY.pub ]]; then
	#		START="genkey"
	#	fi
	#done
	#if [[ "$START" == "genkey" ]]; then
		# Start OpenSSH to generate keys (blocking), and then immediately stop
		/usr/local/etc/init.d/openssh start
		/usr/local/etc/init.d/openssh stop
	#fi
	for ALG in $ALGORITHMS; do
		KEY=ssh_host_${ALG}_key
	#	if [[ ! -f $STORE/$KEY || ! -f $STORE/$KEY.pub ]]; then
			cp $SSHDIR/$KEY $STORE
			cp $SSHDIR/$KEY.pub $STORE
	#	fi
	done
	# Selective updating also doesn't work, need openssh to check and recreate keys
	# This is so stupid
fi