#!/bin/bash

source config.txt

if [[ ! -d $STORAGE_PATH ]]; then
	mkdir $STORAGE_PATH
fi

if [[ -d "$BUILD_PATH/TrackingCamera" ]]; then
	echo "Extracting build artefacts..."
	if [[ ! -d "$STORAGE_PATH/TrackingCamera" ]]; then
		mkdir "$STORAGE_PATH/TrackingCamera"
	fi
	cp $BUILD_PATH/TrackingCamera/* "$STORAGE_PATH/TrackingCamera"
else
	echo "No build artefacts in data!"
fi