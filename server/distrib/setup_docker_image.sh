#!/bin/bash

if [[ "$EUID" != 0 ]]; then
	echo "May need to run as root unless you're running a rootless docker install!"
fi

DOCKER_IMAGE=astertrack-debian-12-llvm-build-env

docker buildx build --progress=plain -f Dockerfile -t $DOCKER_IMAGE .