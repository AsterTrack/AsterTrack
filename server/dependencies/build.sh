#!/usr/bin/env bash

# Select release or debug mode
MODE="$(echo $1 | tr '[:upper:]' '[:lower:]')"
if [[ $MODE == "sanitize-thread" ]]; then
	echo "Make sure to clean dependency build files before switching between sanitized and non-sanitized builds!"
	export CFLAGS="-fsanitize=thread"
	export CXXFLAGS="-fsanitize=thread"
elif [[ $MODE == "sanitize-address" ]]; then
	echo "Make sure to clean dependency build files before switching between sanitized and non-sanitized builds!"
	export CFLAGS="-fsanitize=address"
	export CXXFLAGS="-fsanitize=address"
fi

DEPENDENCIES="Eigen glfw libusb vrpn"
for DEP in $DEPENDENCIES; do
	pushd buildfiles/$DEP > /dev/null
	echo "====================================================================="
	echo "Downloading, building and installing $DEP..."
	./fetch.sh
	if [[ -f "build.sh" ]]; then
		./build.sh
	fi
	popd > /dev/null
done

if [[ $MODE == "sanitize-thread" ]]; then
	echo "Make sure to clean dependency build files before switching between sanitized and non-sanitized builds!"
elif [[ $MODE == "sanitize-address" ]]; then
	echo "Make sure to clean dependency build files before switching between sanitized and non-sanitized builds!"
fi