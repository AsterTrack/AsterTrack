#!/bin/bash

# Include that handles verifying parameters, loading config.txt, mounting & unmounting
source source/mount_data.sh

echo "Extracting $DATA_FILE from TCE partition..."

# Extract default mydata (user files) here to edit
if [[ -f $TCE_PATH$DATA_FILE ]]; then
	rm -rf $DATA_PATH
	mkdir $DATA_PATH
	tar --numeric-owner -xzpf $TCE_PATH$DATA_FILE -C $DATA_PATH
	# Make sure it's readable by user, but also that SSH keys are protected
	USER=$(who am i | awk '{print $1}')
	sudo chown -R $USER $DATA_PATH
	sudo chmod -R 775 $DATA_PATH
	if [[ -f $DATA_PATH/usr/local/etc/ssh ]]; then
		sudo chmod -R 600 $DATA_PATH/usr/local/etc/ssh
	fi
else
	echo "No $DATA_FILE in the TCE partition to save!"
fi