#!/usr/bin/env bash

# Frozen v1.0.30-prerelease
#FETCH_URL=https://github.com/libusb/libusb/archive/refs/tags/v1.0.29.zip
FETCH_URL=https://github.com/libusb/libusb/archive/102ab657bbc20b97579c477149739a6821a45e7a.zip
FETCH_VERSION=v1.0.30-prerelease
FETCH_NAME=libusb
FETCH_ARCHIVE=source.zip

if [[ -f "source/srcversion" ]]; then
	if [[ $(cat source/srcversion) != $FETCH_VERSION ]]; then
		rm "source/srcversion"
	fi
fi

if [[ ! -f "source/srcversion" ]]; then
	echo Downloading $FETCH_NAME $FETCH_VERSION source
	wget -O $FETCH_ARCHIVE $FETCH_URL -q --show-progress
	if [[ $? -ne 0 ]]; then
		echo Failed to download source!
		rm $FETCH_ARCHIVE
		exit 1
	fi

	if [[ -d "source" ]]; then
		rm -r source
	fi

	echo Unpacking $FETCH_NAME $FETCH_VERSION source
	unzip -q $FETCH_ARCHIVE
	mv $FETCH_NAME-* source

	rm $FETCH_ARCHIVE
	echo $FETCH_VERSION > source/srcversion
	echo Done downloading $FETCH_NAME $FETCH_VERSION!
fi