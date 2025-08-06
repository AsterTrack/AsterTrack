#!/bin/bash

# Include that handles verifying parameters, loading config.txt, mounting & unmounting
source source/mount_data.sh

echo "Extracting data file from TCE partition..."

# Extract default mydata (user files) here to edit
if [[ -f $TCE_PATH/mydata.tgz ]]; then
	rm -rf $DATA_PATH
	mkdir $DATA_PATH
	tar --numeric-owner -xzpf $TCE_PATH/mydata.tgz -C $DATA_PATH
	# Make sure it's readable by user
	USER=$(who am i | awk '{print $1}')
	sudo chown -R $USER $DATA_PATH
	sudo chmod -R 775 $DATA_PATH
else
	echo "No data file in the TCE partition to read!"
fi