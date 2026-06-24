#!/bin/bash

if [[ -z $(sudo docker image list | grep vc4-vpu-qpu) ]]; then
    # Build docker image if not done already
    sudo docker buildx use default
    sudo docker buildx build --load -f env/Dockerfile.vc4 -t vc4-vpu-qpu .
fi

# Run auto-build script in source folder
sudo docker run --rm -v $PWD:/data vc4-vpu-qpu

# Copy resulting binaries into storage for setup
mkdir -p setup/storage/TrackingCamera
cp build/qpu_blob_tiled_min.bin build/vpu_programs.bin setup/storage/TrackingCamera/