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
sudo ln -s /home/tc/sources/vc4asm/build-*/vc4asm /usr/local/bin/vc4asm

echo "Building QPU Program!"
mkdir -p sources/camera/build
cd sources/camera/build
cmake ..
make qpu_programs -j 2
sudo chmod a+rwx qpu_blob_tiled_min.bin ../../../TrackingCamera 
cp qpu_blob_tiled_min.bin ../../../TrackingCamera

cd ../../..

sudo filetool.sh -b