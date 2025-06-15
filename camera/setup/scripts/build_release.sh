#!/bin/sh

cd /home/tc

./vc4asm >> /dev/null 2>&1
if [[ $? == 1 ]]; then
	echo "Already built vc4asm!"
else
	echo "Building vc4asm!"
	cd sources/vc4asm
	cmake .
	cd build-*
	make vc4asm -j 4
	sudo cp vc4asm ../../../
	cd ../../..
fi
sudo rm -f /usr/local/bin/vc4asm
sudo ln -s /home/tc/vc4asm /usr/local/bin/vc4asm

if [[ ! -d TrackingCamera ]]; then
	mkdir TrackingCamera
fi

if [[ -f TrackingCamera/libvcsm.so ]]; then
	echo "Already built libvcsm!"
else
	echo "Building libvcsm!"
	mkdir -p sources/libvcsm/build
	cd sources/libvcsm/build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make
	cd ../../..
	cp sources/libvcsm/build/libvcsm.so TrackingCamera/
fi

echo "Building TrackingCamera!"
mkdir -p sources/camera/build
cd sources/camera/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 1
# Can't use more cores as main.cpp already uses more than 300MB of RAM
# Together with the base usage, it already completely bogs down the system and requires swap to work
cp TrackingCamera_armv6l ../../../TrackingCamera/
cp TrackingCamera_armv7l ../../../TrackingCamera/
cp qpu_blob_tiled_min.bin ../../../TrackingCamera/

cd ../../..

sudo filetool.sh -b