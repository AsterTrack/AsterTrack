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