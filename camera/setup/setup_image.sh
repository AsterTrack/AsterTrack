#!/bin/bash

MODE=$1
IMAGE_FILE=$2
REPO_UPDATE=$3
TRCAM_PATH=${4%/}

if [[ $MODE = *"h"* ]]; then
	echo "Usage: sudo setup_image.sh [MODE] [IMAGE_FILE] [SOURCE_PATH] [REPO_UPDATE]
	MODE: [compile|dev-single|dev|wifi|normal] - default: normal
	IMAGE_FILE: Optionally specify image file name to write to - default: image_%mode%.img
	REPO_UPDATE: Optionally specify 'skip_updates' to use local sources - default: update
	SOURCE_PATH: Optionally specify repository folder to look for camera source code - default: parent directory"
	exit 0
fi

if [[ "$EUID" != 0 ]]; then
    echo "Need sudo to execute!"
    exit 1
fi

RUNTIME_DEP="rpi-vc.tcz libv4l2.tcz libjpeg-turbo.tcz"
COMPILE_DEP="rpi-vc-dev.tcz libjpeg-turbo-dev.tcz cmake.tcz compiletc.tcz "
WIRELESS_DEP="firmware-brcmwifi.tcz firmware-rpi-wifi.tcz wifi.tcz openssh.tcz ntp.tcz"
ZEROCONF_DEP="dbus.tcz avahi.tcz nss-mdns.tcz" # so that hostname is announced, else have to use IP
DEV_DEP="bash.tcz gdb.tcz nano.tcz" # bash only for VS Code remote development
TEST_DEP="i2c-tools.tcz v4l2-utils.tcz libv4l2-dev.tcz"
# ntp for accurate time, super annoying when cmake rebuilds everything because of different file times

if [[ $MODE = "compile" ]]; then
# For a quick compile and offload operation without running TrackingCamera
	CREATE_SWAP=True
	INSTALL_SOURCES=True
	AUTO_BUILD=True
	SHUTDOWN_AFTER_BUILD=True
	AUTOCONNECT_WIFI=True # No reason not to - if it takes too long, check in
	SETUP_SSH=True
	PRUNE_STARTUP=False
	DEPENDENCIES="$RUNTIME_DEP $COMPILE_DEP $DEV_DEP"
	DEFAULT_IMAGE_FILE="image_compile.img"
	TOTAL_IMAGE_SIZE=300M	# Size needed for dev dependencies
	GPU_MEM_SIZE=32M		# Need GPU memory for blob detection (~12MB)
	AUTORUN=False
elif [[ $MODE = "dev-single" ]]; then
# Dev image without running TrackingCamera - just sets up wifi and SSH and waits
	CREATE_SWAP=True
	INSTALL_SOURCES=True
	AUTO_BUILD=False
	AUTOCONNECT_WIFI=True
	SETUP_SSH=True
	PRUNE_STARTUP=False
	DEPENDENCIES="$RUNTIME_DEP $WIRELESS_DEP $ZEROCONF_DEP $COMPILE_DEP $DEV_DEP"
	DEFAULT_IMAGE_FILE="image_dev.img"
	TOTAL_IMAGE_SIZE=300M	# Size needed for dev dependencies
	GPU_MEM_SIZE=32M		# Need GPU memory for blob detection (~12MB)
	AUTORUN=False
elif [[ $MODE = "dev" ]]; then
# Dev image that can be used as a normal camera, but allows compilation and dev work via SSH when desired, and logs to SD card for later analysis
	CREATE_SWAP=True
	INSTALL_SOURCES=True
	AUTO_BUILD=False
	AUTOCONNECT_WIFI=False
	SETUP_SSH=True
	PRUNE_STARTUP=False
	DEPENDENCIES="$RUNTIME_DEP $WIRELESS_DEP $COMPILE_DEP $DEV_DEP"
	DEFAULT_IMAGE_FILE="image_dev.img"
	TOTAL_IMAGE_SIZE=300M	# Size needed for dev dependencies
	GPU_MEM_SIZE=32M		# Need GPU memory for blob detection (~12MB)
	AUTORUN=True
elif [[ $MODE = "wifi" ]]; then
# Normal camera image that has wifi and SSH support enabled when setup via host software - e.g. to read logs, for a future server, etc.
	INSTALL_SOURCES=False
	AUTOCONNECT_WIFI=False
	SETUP_SSH=True
	PRUNE_STARTUP=False
	DEPENDENCIES="$RUNTIME_DEP $WIRELESS_DEP"
	DEFAULT_IMAGE_FILE="image_wifi.img"
	TOTAL_IMAGE_SIZE=120M
	GPU_MEM_SIZE=32M		# Need GPU memory for blob detection (~12MB)
	AUTORUN=True
elif [[ $MODE = "" ]]; then
# Normal camera image, as small and quick as can be, no wifi support
	INSTALL_SOURCES=False
	AUTOCONNECT_WIFI=False
	SETUP_SSH=False
	PRUNE_STARTUP=True
	DEPENDENCIES="$RUNTIME_DEP"
	DEFAULT_IMAGE_FILE="image_minimal.img"
	TOTAL_IMAGE_SIZE=100M
	GPU_MEM_SIZE=32M		# Need GPU memory for blob detection (~12MB)
	AUTORUN=True
else
	echo "Unknown mode $MODE!"
	exit 1
fi

if [[ -z "$IMAGE_FILE" ]]; then
	IMAGE_FILE=$DEFAULT_IMAGE_FILE
fi

# Setup additional configurations (paths / URLs)
source config.txt

# Create new image file
DD_SZ=$(echo $TOTAL_IMAGE_SIZE | sed -n "s/\([0-9]*\)\([GMB]\)/bs=1\2 count=\1/p")
dd if=/dev/zero of=$IMAGE_FILE $DD_SZ status=none

# Map image as loop device
DEVICE_PATH=$(losetup --show -f $IMAGE_FILE)

# Make sure it is unmounted at the end
trap "losetup -d $DEVICE_PATH && echo 'Unmapping loop device!'" SIGINT EXIT
# NOTE: Trap signals overwrite each other, and can't easily be chained.
# So as soon as setup_device.sh traps SIGINT (for user interrupts) and RETURN (for aborts) it replaces this one for SIGINT
# To ensure this is still called exit is called explicitly at the end here
# And all traps in setup_device.sh also call exit after they finished cleaning up, ensuring this one also gets called

if source source/setup_device.sh; then
	exit 0 # All good
fi

echo "Something went wrong, but traps didn't trigger exit! This should never happen!"
exit 1
