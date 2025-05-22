#!/usr/bin/env bash

# Make sure source directory is valid
SRC_PATH=source
if [[ ! -d "$SRC_PATH" ]]; then
	echo "$SRC_PATH does not exist!"
	exit 1
fi

# Create temporary directories
mkdir -p linux/build
mkdir -p linux/install

echo -----------------------------------------
echo Building vrpn
echo -----------------------------------------

# Eradicate scripts searching recursively up the directory structure for a git repo to violate
rm $SRC_PATH/ParseVersion.cmake
touch $SRC_PATH/ParseVersion.cmake

pushd linux/build
cmake ../../$SRC_PATH -DVRPN_USE_STD_CHRONO=ON -DVRPN_USE_GPM_MOUSE=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j4 quat vrpn
# Install just needed for easy access to include files
cmake -DCMAKE_INSTALL_COMPONENT=clientsdk -DCMAKE_INSTALL_PREFIX=../install -P cmake_install.cmake
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

# Try to verify output
if [[ ! -d "linux/build/quat" ]]; then
	echo "Build failed - build/quat does not exist!"
	exit 3
fi

# Copy resulting files to project directory
if [[ "${PWD:(-29)}" != "/dependencies/buildfiles/vrpn" ]]; then
	echo "Not in dependencies/buildfiles/vrpn subfolder, can't automatically install files into project folder!"
	exit 4
fi

echo "Installing dependency files..."

rm -r ../../include/vrpn
mkdir -p ../../include/vrpn
cp -r linux/install/include/* ../../include/vrpn

rm -r ../../lib/linux/vrpn
mkdir -p ../../lib/linux/vrpn
cp -r linux/install/lib/* ../../lib/linux/vrpn
#cp linux/build/quat/libquat.a ../../lib/linux/vrpn
#cp linux/build/libvrpn.a ../../lib/linux/vrpn

echo "Now you can call clean.sh if everything succeeded."