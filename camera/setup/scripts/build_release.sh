#!/bin/sh

cd /home/tc

vc4asm >> /dev/null 2>&1
if [[ $? == 1 ]]; then
	echo "Already built vc4asm!"
else
	echo "Building vc4asm!"
	cd sources/vc4asm
	cmake .
	cd build-*
	make vc4asm -j 4
	sudo cp vc4asm /usr/local/bin/
	cd ../../..
fi

echo "Building TrackingCamera!"
mkdir -p sources/camera/build
cd sources/camera/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 1
# Can't use more cores as main.cpp already uses more than 300MB of RAM
# Together with the base usage, it already completely bogs down the system and requires swap to work
cd ../../..

if [[ ! -d TrackingCamera ]]; then
	mkdir TrackingCamera
fi

cp sources/camera/build/TrackingCamera_armv6l TrackingCamera/
cp sources/camera/build/TrackingCamera_armv7l TrackingCamera/
cp sources/camera/build/qpu_blob_tiled_min.bin TrackingCamera/

sudo filetool.sh -b