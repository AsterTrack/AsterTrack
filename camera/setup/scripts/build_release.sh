#!/bin/sh

cd /home/tc

if [[ ! -d TrackingCamera ]]; then
	mkdir TrackingCamera
fi

echo "Building TrackingCamera!"
mkdir -p sources/camera/build
cd sources/camera/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make TrackingCamera_binaries -j 2
# Can't use more cores as main.cpp already uses more than 300MB of RAM
# Together with the base usage, it already completely bogs down the system and requires swap to work
sudo chmod a+rwx TrackingCamera_* tag.bin ../../../TrackingCamera 
cp TrackingCamera_* tag.bin ../../../TrackingCamera

cd ../../..

sudo filetool.sh -b