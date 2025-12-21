#!/bin/bash

source config.txt

if [[ ! -d $STORAGE_PATH ]]; then
	mkdir $STORAGE_PATH
fi

if [[ -d "$BUILD_PATH/drivers" ]]; then
	echo "Extracting driver build..."
	cp -rf $BUILD_PATH/drivers "$STORAGE_PATH/"
else
	echo "No driver build in data!"
fi

if [[ -d "$BUILD_PATH/TrackingCamera" ]]; then
	echo "Extracting program build..."
	cp -rf $BUILD_PATH/TrackingCamera "$STORAGE_PATH/"
else
	echo "No program build in data!"
fi