#!/bin/bash

PROJ_DIR="$(realpath $(dirname "$0")/../..)"
if [[ ! -d "$PROJ_DIR/.git" ]]; then
    echo "Can't find main git directory at '$PROJ_DIR/.git'!"
    exit 1
fi

ROOT_DIR=$1
if [[ -z $ROOT_DIR ]]; then
	ROOT_DIR=debian-chroot
fi

BUILD_TYPE=$2
if [[ -z $BUILD_TYPE ]]; then
	BUILD_TYPE=release
fi

BUILD_CONFIG=$3
if [[ -z $BUILD_CONFIG ]]; then
	BUILD_CONFIG=clang-libc-static
fi

DEBIAN=bookworm

if [[ ! -d debootstrap ]]; then
	git clone https://salsa.debian.org/installer-team/debootstrap.git
fi

if [[ ! -d $ROOT_DIR ]]; then
	mkdir $ROOT_DIR
	sudo DEBOOTSTRAP_DIR=debootstrap debootstrap/debootstrap --arch amd64 $DEBIAN $ROOT_DIR
fi

sudo mount -t proc /proc $ROOT_DIR/proc
sudo mount --rbind /sys $ROOT_DIR/sys
sudo mount --make-rslave $ROOT_DIR/sys
sudo mount --rbind /dev $ROOT_DIR/dev
sudo mount --make-rslave $ROOT_DIR/dev

trap "sudo umount -R $ROOT_DIR/proc $ROOT_DIR/sys $ROOT_DIR/dev" SIGINT EXIT

# Update source code without disturbing existing build files
test -d "$ROOT_DIR/AsterTrack/.git" && rm -rf "$ROOT_DIR/AsterTrack/.git"
mkdir -p "$ROOT_DIR/AsterTrack"
cp -r "$PROJ_DIR/.git" "$ROOT_DIR/AsterTrack/"
pushd "$ROOT_DIR/AsterTrack/"
git restore .
popd

cp setup_toolchain_llvm.sh $ROOT_DIR/

# Interactive shell
#sudo chroot $ROOT_DIR env PATH=/usr/local/sbin:/usr/sbin:/usr/local/bin:/usr/bin bash
#exit 0

# Auto-build LLVM, Dependencies and AsterTrack
sudo chroot $ROOT_DIR env PATH=/usr/local/sbin:/usr/sbin:/usr/local/bin:/usr/bin bash -c "
/setup_toolchain_llvm.sh

apt-get update -y
apt-get install -y autoconf automake libtool unzip wget
apt-get install -y libgl1-mesa-dev libglu1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev libudev-dev libdbus-1-dev libturbojpeg0-dev

export PATH=/opt/toolchain/bin:\$PATH
export CC=/opt/toolchain/bin/clang
export CXX=/opt/toolchain/bin/clang++

# Build AsterTrack dependencies
cd AsterTrack/server/
pushd dependencies
./clean.sh
./build.sh
popd

# Build AsterTrack application
make BUILD_CONFIG=$BUILD_CONFIG BUILD_TYPE=$BUILD_TYPE -j\$(nproc --ignore=2)
"

BUILD_DIR=$PROJ_DIR/server/build
TARGET_DIR=$BUILD_DIR/CHROOT-$BUILD_CONFIG-$BUILD_TYPE
mkdir -p $TARGET_DIR
sudo cp $ROOT_DIR/AsterTrack/server/build/$BUILD_CONFIG-$BUILD_TYPE/astertrack-* $TARGET_DIR/
sudo cp $ROOT_DIR/AsterTrack/server/build/libusb* $TARGET_DIR/
sudo chown -R $(who am i | awk '{print $1}') $TARGET_DIR/

rm -f $BUILD_DIR/astertrack*
ln -sr $TARGET_DIR/astertrack* $BUILD_DIR/
cp -u $TARGET_DIR/libusb* $BUILD_DIR/