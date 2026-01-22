#!/usr/bin/env bash

if [[ -d package ]]; then
    rm -rf package/*
fi
mkdir -p package

cp -r licenses package/

mkdir -p package/resources
cp -r resources/icons package/resources/
cp -r resources/fonts package/resources/
cp resources/astertrack_icon_release.png package/resources/astertrack_icon.png
cp resources/astertrack_icon_release.svg package/resources/astertrack_icon.svg

mkdir -p package/store
cp store/lens_presets_builtin.json package/store
cp store/general_config.json package/store
cp store/camera_simulated.json package/store
cp store/Target_Ring.obj package/store
cp store/Target_Sparse.obj package/store

cp build/astertrack-server package
cp build/astertrack-interface.so package
cp build/libusb-1.* package

cp README.md package
cp astertrack.desktop package

# Copy and edit so only appended install script remains (with correct executable bit)
cp $0 package/install.sh
match=$(grep --text --line-number '^exit 0 # Install Script Below$' $0 | cut -d ':' -f 1)
sed -i 1,"$match"d package/install.sh

echo "Finished packaging program into package folder!"

# Don't want the install.sh in the repo root to prevent it being executed where it should not be
exit 0 # Install Script Below
#!/usr/bin/env bash

INSTALL_DIR=$1
DESKTOP_DIR=
ICON_DIR=

if [[ "$EUID" == 0 ]]; then
    echo "System installation currently not supported, do not use sudo!"
    exit 1
    if [[ -z $INSTALL_DIR ]]; then
        INSTALL_DIR=/opt/astertrack
    fi
    DESKTOP_DIR=/usr/share/applications
    ICON_DIR=/usr/share/icons/hicolor/scalable/apps
else
    if [[ -z $INSTALL_DIR ]]; then
        INSTALL_DIR=$HOME/.local/share/astertrack
    fi
    DESKTOP_DIR=$HOME/.local/share/applications
    ICON_DIR=/usr/local/share/icons/hicolor/scalable/apps
fi

if [[ $INSTALL_DIR != *"astertrack"* && $INSTALL_DIR != *"AsterTrack"* ]]; then
    echo "Installing in unusual directory '$INSTALL_DIR', expected to contain 'astertrack'!"
    exit 1
fi

echo "Installing to '$INSTALL_DIR'..."

if [[ -d $INSTALL_DIR ]]; then
    if [[ $INSTALL_DIR != *"astertrack"* && $INSTALL_DIR != *"AsterTrack"* ]]; then
        echo "Cannot clean prior installation!"
    else
        rm -rf $INSTALL_DIR/*
    fi
fi

# Copy all program files and initial configs to install directory
mkdir -p $INSTALL_DIR || exit 1
cp -r $(dirname $0)/* $INSTALL_DIR/

# Fix absolute path in .desktop file before installing it
sed -i s,/opt/astertrack,$INSTALL_DIR,g $INSTALL_DIR/astertrack.desktop
cp $INSTALL_DIR/astertrack.desktop $DESKTOP_DIR/

echo "Done installing to '$INSTALL_DIR'!"