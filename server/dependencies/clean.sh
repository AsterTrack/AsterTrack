#!/usr/bin/env bash

MODE=build # all, build
if [[ ! -z $1 ]]; then
	MODE=$1
fi

DEPENDENCIES="Eigen glfw libusb vrpn"
for DEP in $DEPENDENCIES; do
	pushd buildfiles/$DEP
	if [[ -f "clean.sh" ]]; then
		./clean.sh $MODE
	fi
	popd
done