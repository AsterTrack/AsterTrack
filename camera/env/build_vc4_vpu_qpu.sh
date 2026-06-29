#!/bin/bash

if [[ -z $(sudo docker image list | grep vc4-vpu-qpu) ]]; then
    # Build docker image if not done already
    sudo docker buildx use default
    sudo docker buildx build --load -f env/Dockerfile.vc4 -t vc4-vpu-qpu .
fi

# Run auto-build script in source folder
sudo docker run --rm -v $PWD:/sources/camera -v $PWD/../shared:/sources/shared vc4-vpu-qpu

# Copy resulting binaries into storage for setup
mkdir -p setup/storage/TrackingCamera
cp build-vc4/qpu_blob_tiled_min.bin build-vc4/vpu_programs.bin setup/storage/TrackingCamera/