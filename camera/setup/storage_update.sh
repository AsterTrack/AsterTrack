#!/bin/bash

if [[ "$EUID" != 0 ]]; then
    echo "Need sudo to execute!"
    exit 1
fi

source config.txt

echo "Extracting wifi and build files..."

if [[ ! -d $STORAGE_PATH ]]; then
	mkdir $STORAGE_PATH
fi

if [[ -f "$HOME_PATH/wifi.db" ]]; then
	cp "$HOME_PATH/wifi.db" $STORAGE_PATH
fi
if [[ -f "$DATA_PATH/usr/local/vc4asm" ]]; then
	cp "$DATA_PATH/usr/local/vc4asm" $STORAGE_PATH
fi
if [[ -d "$BUILD_PATH/TrackingCamera" ]]; then
	if [[ ! -d "$STORAGE_PATH/TrackingCamera" ]]; then
		mkdir "$STORAGE_PATH/TrackingCamera"
	fi
	cp $BUILD_PATH/TrackingCamera/TrackingCamera* "$STORAGE_PATH/TrackingCamera"
	cp $BUILD_PATH/TrackingCamera/*bin "$STORAGE_PATH/TrackingCamera"
fi