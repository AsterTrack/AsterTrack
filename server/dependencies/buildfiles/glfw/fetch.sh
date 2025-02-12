#!/usr/bin/env bash

FETCH_URL=https://github.com/Seneral/glfw/archive/9a1bc9b1bf422c1aa2b32e7821e382f796fa8f28.zip
FETCH_VERSION=3.4.0+WL
FETCH_NAME=glfw
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