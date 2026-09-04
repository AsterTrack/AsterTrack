#!/bin/bash

PROJ_DIR="$(realpath $(dirname "$0")/../..)"
if [[ ! -d "$PROJ_DIR/.git" ]]; then
    echo "Can't find main git directory at '$PROJ_DIR/.git'!"
    exit 1
fi

ROOT_DIR=$1
if [[ -z $ROOT_DIR ]]; then
	ROOT_DIR=debian-docker
fi

BUILD_TYPE=$2
if [[ -z $BUILD_TYPE ]]; then
	BUILD_TYPE=release
fi

BUILD_CONFIG=$3
if [[ -z $BUILD_CONFIG ]]; then
	BUILD_CONFIG=clang-libc-static
fi

DOCKER_IMAGE=astertrack-debian-12-llvm-build-env

if [ -z "$(docker images -q $DOCKER_IMAGE 2> /dev/null)" ]; then
    ./setup_docker_image.sh
fi

if [ -z "$(docker images -q $DOCKER_IMAGE 2> /dev/null)" ]; then
    exit 1
fi

# Update source code without disturbing existing build files
test -d "$ROOT_DIR/AsterTrack/.git" && rm -rf "$ROOT_DIR/AsterTrack/.git"
mkdir -p "$ROOT_DIR/AsterTrack"
cp -r "$PROJ_DIR/.git" "$ROOT_DIR/AsterTrack/"
pushd "$ROOT_DIR/AsterTrack/"
git restore .
popd

# Run build in docker container
docker run --rm -v "$(pwd)/$ROOT_DIR/AsterTrack":/AsterTrack -w "/" --entrypoint /bin/bash $DOCKER_IMAGE -c "

# Build AsterTrack dependencies
cd AsterTrack/server/
pushd dependencies
./clean.sh
./build.sh
popd

# Build AsterTrack application
make BUILD_CONFIG=$BUILD_CONFIG BUILD_TYPE=$BUILD_TYPE -j\$(nproc --ignore=2)
" $@

BUILD_DIR=$PROJ_DIR/server/build
TARGET_DIR=$BUILD_DIR/DOCKER-$BUILD_CONFIG-$BUILD_TYPE/
mkdir -p $TARGET_DIR
cp $ROOT_DIR/AsterTrack/server/build/$BUILD_CONFIG-$BUILD_TYPE/astertrack-* $TARGET_DIR
cp $ROOT_DIR/AsterTrack/server/build/libusb* $TARGET_DIR
if [[ "$EUID" == 0 ]]; then
    chown -R $(who am i | awk '{print $1}') $TARGET_DIR
fi

rm -f $BUILD_DIR/astertrack*
ln -sr $TARGET_DIR/astertrack* $BUILD_DIR/
cp -u $TARGET_DIR/libusb* $BUILD_DIR/