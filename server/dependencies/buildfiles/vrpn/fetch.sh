#!/usr/bin/env bash

# Frozen v07.36
#FETCH_URL=https://github.com/vrpn/vrpn/archive/refs/tags/v07.36.zip
# Custom modification to expose more connectivity information
FETCH_URL=https://github.com/Seneral/vrpn/archive/3474fff0c3bb12b56af16a5f7ed469558e54a597.zip
FETCH_VERSION=v07.37-custom
FETCH_NAME=vrpn
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