#!/usr/bin/env bash

#FETCH_URL=https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
# Need newer version than old 3.4.0, change to fixed version as soon as one is released
#FETCH_URL=https://gitlab.com/libeigen/eigen/-/archive/master/eigen-master.zip
FETCH_URL=https://gitlab.com/libeigen/eigen/-/archive/8ac2fb077dfcdf9a6fa78a871c494594dfa795b0/eigen-8ac2fb077dfcdf9a6fa78a871c494594dfa795b0.zip
FETCH_VERSION=master-8ac2fb07
FETCH_NAME=eigen
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

# Copy resulting files to project directory
if [[ "${PWD:(-30)}" != "/dependencies/buildfiles/Eigen" ]]; then
	echo "Not in dependencies/buildfiles/Eigen subfolder, can't automatically install files into project folder!"
	exit 2
fi

echo "Installing dependency files..."

rm -rf ../../include/Eigen
rm -rf ../../include/unsupported
mkdir -p ../../include/unsupported
cp -r source/Eigen ../../include
cp -r source/unsupported/Eigen ../../include/unsupported

echo "Now you can call clean.sh if everything succeeded."
exit 0