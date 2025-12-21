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

echo "Building TrackingCamera!"
mkdir -p sources/camera/build-debug
cd sources/camera/build-debug
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON ..
make TrackingCamera_$(uname -m) -j 2
# Can't use more cores as main.cpp already uses more than 300MB of RAM
# Together with the base usage, it already completely bogs down the system and requires swap to work
sudo chmod a+rwx ../../../TrackingCamera TrackingCamera_* qpu_blob_tiled_min.bin
cp TrackingCamera_* qpu_blob_tiled_min.bin ../../../TrackingCamera/

cd ../../..

sudo filetool.sh -b