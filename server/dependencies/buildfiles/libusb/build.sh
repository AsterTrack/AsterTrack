#!/usr/bin/env bash

# Make sure source directory is valid
SRC_PATH=source
if [[ ! -d "$SRC_PATH" ]]; then
	echo "$SRC_PATH does not exist!"
	exit 1
fi

# Create temporary directories
mkdir -p linux/install
mkdir -p linux/build
ABS_INSTALL_PATH=$(pwd)/linux/install

echo -----------------------------------------
echo Building libusb
echo -----------------------------------------

pushd $SRC_PATH
./bootstrap.sh
popd
pushd linux/build
../../$SRC_PATH/configure --prefix=$ABS_INSTALL_PATH --enable-shared=yes --enable-static=no --disable-log
make install -j4
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

# Try to verify output
if [[ ! -d "linux/install/lib" ]]; then
	echo "Build failed - install/libs does not exist!"
	exit 3
fi

# Copy resulting files to project directory
if [[ "${PWD:(-31)}" != "/dependencies/buildfiles/libusb" ]]; then
	echo "Not in dependencies/buildfiles/libusb subfolder, can't automatically install files into project folder!"
	exit 4
fi

echo "Installing dependency files..."

rm -r ../../include/libusb
mkdir -p ../../include/libusb
cp -r linux/install/include/libusb-*/* ../../include/libusb

rm -r ../../lib/linux/libusb
mkdir -p ../../lib/linux/libusb
cp -r linux/install/lib/* ../../lib/linux/libusb

echo "Now you can call clean.sh if everything succeeded."