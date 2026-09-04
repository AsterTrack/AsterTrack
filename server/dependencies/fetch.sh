#!/usr/bin/env bash

set -e

DEPENDENCIES="Eigen glfw libusb vrpn"
for DEP in $DEPENDENCIES; do
	pushd buildfiles/$DEP > /dev/null
	echo "====================================================================="
	echo "Downloading $DEP..."
	./fetch.sh
	popd > /dev/null
done