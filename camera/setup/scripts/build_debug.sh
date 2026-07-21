#!/bin/sh

if [[ "$EUID" == 0 ]]; then
	echo "Don't run as root, will mess up build folder!"
	exit 1
fi

tce-load -li $(cat /mnt/mmcblk0p2/tce/oncompile.lst) >/dev/null

cd /home/tc

if [[ ! -d TrackingCamera ]]; then
	mkdir TrackingCamera
fi

echo "Building TrackingCamera!"
mkdir -p sources/camera/build-debug
cd sources/camera/build-debug
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON ..
make TrackingCamera_$(uname -m) -j 2
#make -j 2 # Build all, e.g. to build for Zero 1 debugging on a Zero 2
# Can't use more cores as main.cpp already uses more than 300MB of RAM
# Together with the base usage, it already completely bogs down the system and requires swap to work
sudo chmod a+rwx TrackingCamera_* tag.bin ../../../TrackingCamera 
cp TrackingCamera_* tag.bin ../../../TrackingCamera

cd ../../..

sudo filetool.sh -b