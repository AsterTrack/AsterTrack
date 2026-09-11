#!/usr/bin/env bash

PROJ_DIR="$(realpath $(dirname "$0")/../..)"
if [[ ! -d "$PROJ_DIR/.git" ]]; then
    echo "Can't find main git directory at '$PROJ_DIR/.git'!"
    exit 1
fi

if [[ -d package ]]; then
    rm -rf package
fi
mkdir -p package
pushd package

server=$PROJ_DIR/server

cp -r $server/licenses .

mkdir -p resources
cp -r $server/resources/icons resources/
cp -r $server/resources/fonts resources/
cp $server/resources/astertrack_icon_release.png resources/astertrack_icon.png
cp $server/resources/astertrack_icon_release.svg resources/astertrack_icon.svg

mkdir -p config
cp $server/config/lens_presets_builtin.json config
cp $server/config/general_config.json config
cp $server/config/camera_simulated.json config
cp $server/config/Target_Ring.obj config
cp $server/config/Target_Sparse.obj config

mkdir -p store/trackers

cp $server/build/astertrack-server .
cp $server/build/astertrack-interface.so .
cp $server/build/libusb-1.* .

cp $server/README.md .
cp $server/distrib/astertrack.desktop .
cp $server/distrib/40-astertrack.rules .

cp $server/distrib/install.sh install.sh

popd

echo "Finished packaging program into package folder!"