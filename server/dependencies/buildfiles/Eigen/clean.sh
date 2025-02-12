#!/usr/bin/env bash

MODE="$(echo $1 | tr '[:upper:]' '[:lower:]')"
if [[ $MODE != build ]]; then
	echo Cleaning source
	rm -r source
fi