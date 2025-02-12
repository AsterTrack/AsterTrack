#!/usr/bin/env bash

MODE="$(echo $1 | tr '[:upper:]' '[:lower:]')"
if [[ $MODE == build ]]; then
	echo Cleaning linux builds
	rm -r linux
else
	echo Cleaning all builds and sources
	rm -r win
	rm -r linux
	rm -r source
fi