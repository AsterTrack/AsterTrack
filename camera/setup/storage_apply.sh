#!/bin/bash

if [[ "$EUID" != 0 ]]; then
    echo "Need sudo to execute!"
    exit 1
fi

source config.txt

echo "Extracting wifi and build files..."

if [[ ! -d $STORAGE_PATH ]]; then
	echo "No storage folder to copy from"
fi

# Copy wifi credentials if exists
if [[ -f $WIFI_DB ]]; then
	cp $WIFI_DB $HOME_PATH
fi

# Copy existing build files if they exist
if [[ -d "$STORAGE_PATH/TrackingCamera" ]]; then
	cp -r $STORAGE_PATH/TrackingCamera $BUILD_PATH
fi

# Copy existing vc4asm binary if it exists
if [[ -f "$STORAGE_PATH/vc4asm" ]]; then
	mkdir -p "$DATA_PATH/usr/local/bin"
	cp "$STORAGE_PATH/vc4asm" "$DATA_PATH/usr/local/bin"
fi