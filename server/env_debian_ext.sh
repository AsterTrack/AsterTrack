#!/bin/bash

rootdir=$1
if [[ -z $rootdir ]]; then
	rootdir=debian-internal
fi

if [[ ! -d debootstrap ]]; then
	git clone https://salsa.debian.org/installer-team/debootstrap.git
fi

if [[ ! -d $rootdir ]]; then
	mkdir $rootdir
	pushd debootstrap
	DEBOOTSTRAP_DIR=. ./debootstrap --arch amd64 bookworm ../$rootdir
	popd
fi

sudo mount -t proc /proc $rootdir/proc
sudo mount --rbind /sys $rootdir/sys
sudo mount --make-rslave $rootdir/sys
sudo mount --rbind /dev $rootdir/dev
sudo mount --make-rslave $rootdir/dev

trap "sudo umount -R $rootdir/proc $rootdir/sys $rootdir/dev" SIGINT EXIT

sudo chroot $rootdir env PATH=/usr/local/sbin:/usr/sbin:/usr/local/bin:/usr/bin bash -c "
apt install debian-archive-keyring
apt update && apt upgrade
apt install -y gcc g++ libomp-dev pkg-config
apt install -y libgl1-mesa-dev libglu1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev libudev-dev libdbus-1-dev libturbojpeg0-dev

cp /usr/lib/llvm-*/lib/clang/*/include/omp.h /usr/include"
