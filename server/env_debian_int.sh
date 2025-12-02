#!/bin/bash

rootdir=$1
if [[ -z $rootdir ]]; then
	rootdir=debian-external
fi

BUILD_TYPE=$2
if [[ -z $BUILD_TYPE ]]; then
	BUILD_TYPE=release
fi

#rm -rf debootstrap
#rm -rf $rootdir
#test -d $rootdir/llvm-project && rm -rf $rootdir/llvm-project
#test -d $rootdir/AsterTrack && rm -rf $rootdir/AsterTrack
#test -d $rootdir/AsterTrack/server/dependencies/lib && rm -rf $rootdir/AsterTrack/server/dependencies/lib
#test -d $rootdir/AsterTrack/server/dependencies/buildfiles && rm -rf $rootdir/AsterTrack/server/dependencies/buildfiles
#test -d $rootdir/AsterTrack/server/build && rm -rf $rootdir/AsterTrack/server/build

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

# Update source code without disturbing existing build files
test -d $rootdir/AsterTrack/.git && rm -rf $rootdir/AsterTrack/.git
mkdir -p $rootdir/AsterTrack
cp -r $(dirname "$0")/../.git $rootdir/AsterTrack/
pushd $rootdir/AsterTrack/
git restore .
popd

sudo chroot $rootdir env PATH=/usr/local/sbin:/usr/sbin:/usr/local/bin:/usr/bin bash -c "
apt install debian-archive-keyring
apt update && apt upgrade
apt install -y gcc g++ ninja-build git cmake python3 binutils-dev

git config --global --add remote.origin.fetch '^refs/heads/users/*'
git config --global --add remote.origin.fetch '^refs/heads/revert-*'
git clone --depth 1 --branch llvmorg-21.1.7 https://github.com/llvm/llvm-project.git

pushd llvm-project

GCCVER=\$(gcc -dumpversion | cut -f1 -d.)
cmake -S llvm -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
	-DLLVM_ENABLE_PROJECTS='clang;lld' -DLLVM_ENABLE_RUNTIMES='libcxx;openmp' \
	-DLIBCXX_CXX_ABI=libstdc++ \
	-DLIBCXX_CXX_ABI_INCLUDE_PATHS=\"/usr/include/c++/\$GCCVER/;/usr/include/\$(gcc -dumpmachine)/c++/\$GCCVER/\" \
	-DLLVM_BINUTILS_INCDIR=/usr/include
# For other BUILD_TYPES (especially verify/debug ones), other runtimes may be interesting:
# -DLLVM_ENABLE_RUNTIMES='libcxx;compiler-rt;openmp;libunwind'

ninja -C build clang lld install
ninja -C build runtimes install-runtimes

popd

apt install -y autoconf automake libtool unzip wget
apt install -y libgl1-mesa-dev libglu1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev libudev-dev libdbus-1-dev libturbojpeg0-dev

apt install -y libomp-dev
cp /usr/lib/llvm-*/lib/clang/*/include/omp.h /usr/include

export CC=/usr/local/bin/clang
export CXX=/usr/local/bin/clang++

cd AsterTrack/server/
pushd dependencies
./build.sh
popd

make COMPILER=clang-std BUILD_TYPE=$BUILD_TYPE -j\$(nproc --ignore=2)
"

TGT_BUILD=build/INT-clang-std-$BUILD_TYPE/
mkdir -p $TGT_BUILD
sudo cp $rootdir/AsterTrack/server/build/clang-std-$BUILD_TYPE/astertrack-* $TGT_BUILD
sudo cp $rootdir/AsterTrack/server/build/libusb* $TGT_BUILD
sudo chown -R $(who am i | awk '{print $1}') $TGT_BUILD
sudo cp $TGT_BUILD* build/
