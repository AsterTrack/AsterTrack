#!/bin/bash

# Include that handles verifying parameters, loading config.txt, mounting & undmounting
source source/mount_data.sh

echo "Writing $DATA_FILE to TCE partition..."

# Make sure it's usable on the device and SSH keys have required levels of safety
sudo chown -R root $DATA_PATH
sudo chmod -R 777 $DATA_PATH
sudo chmod -R 600 $DATA_PATH/usr/local/etc/ssh

pushd $DATA_PATH > /dev/null # because tar is a terribly inconsistent command
tar -czpf $TCE_PATH/$DATA_FILE --numeric-owner *
popd > /dev/null

# Make sure it's again readable by user, but also that SSH keys are protected
USER=$(who am i | awk '{print $1}')
sudo chown -R $USER $DATA_PATH
sudo chmod -R 775 $DATA_PATH
sudo chmod -R 600 $DATA_PATH/usr/local/etc/ssh
