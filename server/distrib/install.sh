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

# Installing udev-rules requires sudo
NEW_RULES=$(dirname $0)/40-astertrack.rules
OLD_RULES=/etc/udev/rules.d/40-astertrack.rules
if ! cmp -s $NEW_RULES $OLD_RULES; then
    echo "Updating udev rules for AsterTrack controller!"
    # Does not work, execution from file browser not supported
    sudo --askpass -p="Need privileges to update udev rules for AsterTrack controller" echo "+" > /dev/null || exit 1
    sudo cp $NEW_RULES $OLD_RULES
    sudo udevadm control --reload-rules && sudo udevadm trigger
fi
if [[ -z "$(getent group | grep sysplugdev)" ]]; then
    echo "Adding sysplugdev group for user to access controller udev rules!"
    # Does not work, execution from file browser not supported
    sudo --askpass -p="Need privileges to add sysplugdev group for user to access controller udev rules" echo "+" > /dev/null || exit 1
    sudo groupadd --system sysplugdev
    sudo usermod -a -G sysplugdev $USER
fi
if [[ -z "$(getent group | grep sysplugdev)" ]]; then
    echo "Failed to add sysplugdev group for user to access controller udev rules!"
fi
