trap "exit 1" SIGINT RETURN

if [[ -z "$DEVICE_PATH" || -z "$MOUNT_DATA" ]]; then
	echo "Do not call this script directly, variables and parameters are missing!"
	return 1
fi

if [[ "$EUID" != 0 ]]; then
	echo "Need sudo to execute!"
	return 1
fi

PARTITION_BOOT=$DEVICE_PATH"p1"
PARTITION_DATA=$DEVICE_PATH"p2"

# Make sure all partitions are unmounted
# With a loop device (from setup_image) this is given
umount -q -f $DEVICE_PATH
if [[ -b $PARTITION_BOOT ]]; then
	umount -q -f $PARTITION_BOOT
fi
if [[ -b $PARTITION_DATA ]]; then
	umount -q -f $PARTITION_DATA
fi

# Ensure required folders exist
mkdir -p $DOWNLOAD_PATH

IMAGE_PATH="$DOWNLOAD_PATH/piCore-$VERSION_SPECIFIC.img"
if [[ ! -f $IMAGE_PATH ]]; then
	echo "Downloading OS image..."
	# Download and unzip
	IMAGE_ZIP_PATH="piCore-$VERSION_SPECIFIC.zip"
	wget -N -q -P $DOWNLOAD_PATH --show-progress "$MAIN_REPO_URL/releases/RPi/$IMAGE_ZIP_PATH"
	if [[ ! -f "$DOWNLOAD_PATH/$IMAGE_ZIP_PATH" ]]; then
		echo "Failed to download piCore image $IMAGE_ZIP_PATH!"
		return 1
	fi
	unzip -q -o "$DOWNLOAD_PATH/$IMAGE_ZIP_PATH" -d $DOWNLOAD_PATH
	rm "$DOWNLOAD_PATH/$IMAGE_ZIP_PATH"
	if [[ ! -f $IMAGE_PATH ]]; then
		echo "Failed to download or extract piCore-$VERSION_SPECIFIC.img!"
		return 1
	fi
fi

echo "Writing OS image..."

# Write default piCore image
dd bs=4M if=$IMAGE_PATH of=$DEVICE_PATH status=none
if [[ $? == 1 ]]; then
	echo "Failed to write to $DEVICE_PATH!"
	return 1
fi
# Make sure kernel knows about the partitions to we can address them with p subscript
sudo partprobe $DEVICE_PATH

echo "Preparing OS..."

# Make sure mounting points are clear
if [[ -d $MOUNT_DATA ]]; then
	umount -q -f $MOUNT_DATA
	rmdir $MOUNT_DATA
fi
if [[ -d $MOUNT_DATA ]]; then
	echo "Data mounting path is not empty!"
	return 1
fi
mkdir $MOUNT_DATA
if [[ -d $MOUNT_BOOT ]]; then
	umount -q -f $MOUNT_BOOT
	rmdir $MOUNT_BOOT
fi
if [[ -d $MOUNT_BOOT ]]; then
	echo "Boot mounting path is not empty!"
	return 1
fi
mkdir $MOUNT_BOOT

# Prune startup partition if desired (remove RPi-4 support)
#if [[ $PRUNE_STARTUP = "True" ]]; then
	#mount $PARTITION_BOOT $MOUNT_BOOT

	#ls $MOUNT_BOOT

	#resize2fs $PARTITION_BOOT # > /dev/null 2>&1
#fi

#echo "Partitions: $(parted --align opt --script $DEVICE_PATH print)"
#ls $PARTITION_BOOT

#echo "Testing resize:"
#parted --align opt --script $DEVICE_PATH resizepart 2 $TOTAL_IMAGE_SIZE
#ls $PARTITION_BOOT

#echo "Partitions: $(parted --align opt --script $DEVICE_PATH print)"
#return 0

# Need to resize the data partition to hold packages and data
parted --align opt --script $DEVICE_PATH resizepart 2 $TOTAL_IMAGE_SIZE
if [[ $? == 1 ]]; then
	echo "Failed to resize TCE partition on $DEVICE_PATH!"
	return 1
fi

# Fix filesystem to fit partition size
resize2fs -f $PARTITION_DATA > /dev/null 2>&1 # if not f, it will ask to run e2fsck, which will create lost+found folder. Skip.
if [[ $? == 1 ]]; then
	echo "Failed to resize filesystem for TCE partition on $DEVICE_PATH!"
	return 1
fi

# Mount data partition
mount $PARTITION_DATA $MOUNT_DATA;
if [[ $? != 0 ]] then
	echo "Failed mounting data partition $PARTITION_DATA at $MOUNT_DATA!"
	return 1
fi
trap "umount -q -f $MOUNT_DATA && echo 'Unmounting TCE partition...' && exit 1" SIGINT RETURN
# NOTE on trap usage: DON'T overwrite EXIT, always make sure exit is called at the end, always keep chaining onto last trap

# Mount boot partition
mount $PARTITION_BOOT $MOUNT_BOOT;
if [[ $? != 0 ]] then
	echo "Failed mounting boot partition $PARTITION_BOOT at $MOUNT_BOOT!"
	return 1
fi
trap "umount -q -f $MOUNT_BOOT && umount -q -f $MOUNT_DATA && echo 'Unmounting TCE and Boot partition...' && exit 1" SIGINT RETURN

# Extract default mydata (user files) here to edit
if [[ -f $TCE_PATH/mydata.tgz ]]; then
	rm -rf $DATA_PATH
	mkdir $DATA_PATH
	tar --numeric-owner -xzpf $TCE_PATH/mydata.tgz -C $DATA_PATH
	USER=$(who am i | awk '{print $1}')
	chown -R $USER $DATA_PATH
	chmod -R 775 $DATA_PATH
	chmod -R 600 $DATA_PATH/usr/local/etc/ssh
else
	echo "No mydata.tgz in the TCE partition! Need template in $DATA_PATH folder!"
	if [[ ! -d $DATA_PATH ]]; then
		echo "No template folder $DATA_PATH found!"
		return 1
	fi
fi

echo "Configuring OS..."

# Disable unnecessary config.txt lines
sed -i "s|dtparam=audio=on$|#dtparam=audio=on|" "$MOUNT_BOOT/config.txt"

#arm_freq=600
#arm_freq_min=200
#arm_freq_max=800
#over_voltage=-8
#over_voltage_min=-8
#gpu_freq=200
echo "
[ALL]
arm_freq=1000
arm_freq_min=800
arm_freq_max=1000
# Sets all unused GPU parts to 250MHz
gpu_freq_min=250
gpu_freq=250
# V3d is default 250-300MHz
v3d_freq_min=300
v3d_freq=400
#v3d_freq=350
#v3d_freq=300
# Core is default 400MHz
core_freq_min=400
core_freq=500
#core_freq=400
# SDRam default is 400-450MHz
sdram_freq_min=450
sdram_freq=500
#sdram_freq=450

#over_voltage=0
#over_voltage_min=-4
# Increase GPU memory (256 is fine)
gpu_mem=$GPU_MEM_SIZE
#gpu_mem=368
#cma_lwm=16
#cma_hwm=32
#cma_offline_start=16
# Enable access to camera I2C10, used to configure the OV9281
dtparam=i2c_vc=on
# Load built-in Kernel driver for OV9281 camera
dtoverlay=ov9281,media-controller=0
# Alternatively switch bt to mini-uart and fix VC frequency to 300Mhz, but bt will be slow
dtoverlay=disable-bt
# Init PL011 clock to allow up to 3MBaud
#init_uart_clock=48000000
# Init PL011 clock to allow up to 9MBaud
init_uart_clock=144000000
# Setup I2C1 for communicating with the STM32 bootloader
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
" >> $MOUNT_BOOT/config.txt

# Copy backup to package path
cp $MOUNT_BOOT/config.txt $STORAGE_PATH/config.txt

#echo "console=tty1 root=/dev/ram0 elevator=deadline rootwait quiet nortc loglevel=3 noembed nozswap" > $MOUNT_BOOT/cmdline.txt

# Copy startup scripts
cp $STARTUP_PATH/* "$DATA_PATH/opt/"
cp $STARTUP_PATH/.filetool.lst "$DATA_PATH/opt/"

# Copy scripts for program interaction and building
cp $SCRIPTS_PATH/* "$BUILD_PATH/"


echo "Copying program and sources..."

if [[ $FORCE_BUILD != "True" ]]; then
	# Copy existing TrackingCamera build if it exists
	if [[ -d "$STORAGE_PATH/TrackingCamera" ]]; then
		cp -r $STORAGE_PATH/TrackingCamera $BUILD_PATH/
	fi
fi

# Download TrackingCamera sources if not provided
if [[ ! -d "$TRCAM_PATH" ]]; then
	TRCAM_PATH=../..
fi

# Copy TrackingCamera sources
mkdir -p $BUILD_PATH/sources/camera
cp -r $TRCAM_PATH/camera/source $BUILD_PATH/sources/camera/
cp -r $TRCAM_PATH/camera/dependencies $BUILD_PATH/sources/camera/
cp -r $TRCAM_PATH/shared $BUILD_PATH/sources/
cp -r $TRCAM_PATH/camera/qpu_programs $BUILD_PATH/sources/camera/
cp -r $TRCAM_PATH/camera/licenses $BUILD_PATH/sources/camera/
cp $TRCAM_PATH/camera/LICENSE $BUILD_PATH/sources/camera/
cp $TRCAM_PATH/camera/README.md $BUILD_PATH/sources/camera/
cp $TRCAM_PATH/camera/CMakeLists.txt $BUILD_PATH/sources/camera/

# Download and copy vc4asm sources
if [[ ! -d "$DOWNLOAD_PATH/vc4asm" ]]; then
	echo "Downloading vc4asm..."
	wget -q --show-progress $VC4ASM_URL -O "$DOWNLOAD_PATH/vc4asm.zip"
	if [[ ! -f "$DOWNLOAD_PATH/vc4asm.zip" ]]; then
		echo "Failed to download vc4asm sources!"
		return 1
	fi
	if [[ -d "$DOWNLOAD_PATH/vc4asm" ]]; then
		rm -r "$DOWNLOAD_PATH/vc4asm"
	fi
	unzip -q "$DOWNLOAD_PATH/vc4asm.zip" -d "$DOWNLOAD_PATH/vc4asm"
	rm "$DOWNLOAD_PATH/vc4asm.zip"
	subdir=$(ls "$DOWNLOAD_PATH/vc4asm")
	mv $DOWNLOAD_PATH/vc4asm/$subdir/* "$DOWNLOAD_PATH/vc4asm"
	mv $DOWNLOAD_PATH/vc4asm/$subdir/.* "$DOWNLOAD_PATH/vc4asm" > /dev/null 2>&1 # Hidden files, but includes ignore . and ..
	rmdir "$DOWNLOAD_PATH/vc4asm/$subdir"
fi

# Copy vc4asm sources
cp -r "$DOWNLOAD_PATH/vc4asm" $BUILD_PATH/sources


# Save data to device

echo "Packing data..."

chown -R root $DATA_PATH
chmod -R 777 $DATA_PATH

CWD=$(pwd)
pushd $DATA_PATH > /dev/null # because tar is a terribly inconsistent command
tar -czpf $CWD/$STORAGE_PATH/mydata.tgz --numeric-owner *
popd > /dev/null

chown -R $USER $DATA_PATH
chmod -R 775 $DATA_PATH

# Change permissions of package data (mydata.tgz and config.txt)
chown $USER $STORAGE_PATH/mydata.tgz $STORAGE_PATH/config.txt
chmod 775 $STORAGE_PATH/mydata.tgz $STORAGE_PATH/config.txt

# Write packed data to device
cp $STORAGE_PATH/mydata.tgz $TCE_PATH/

# Copy existing TrackingCameraMCU build if it exists
if [[ -f "$STORAGE_PATH/TrackingCameraMCU.bin" ]]; then
	cp $STORAGE_PATH/TrackingCameraMCU.bin $TCE_PATH/
fi

# Configure behaviour of image
mkdir -p $MOUNT_DATA/config/

if [[ $AUTOCONNECT_WIFI = "True" ]]; then
	if [[ -f $STORAGE_PATH/wpa_supplicant.conf ]]; then
		echo "Setting up wifi credentials to enable autoconnect..."
		cp $STORAGE_PATH/wpa_supplicant.conf $MOUNT_DATA/config/
		touch $MOUNT_DATA/config/wireless_autoconnect
	else
		echo "Cannot setup wifi autoconnect, missing wifi credentials!"
	fi
fi

if [[ $ENABLE_LOGGING = "True" ]]; then
	touch $MOUNT_DATA/config/log
fi


echo "Installing dependencies..."

# Enter direct dependencies into onboot.lst
for pkg in $DEPENDENCIES; do
	echo $pkg >> $TCE_PATH/onboot.lst
done

pushd $DOWNLOAD_PATH > /dev/null

# Update package list of all used repositories
for arch in "${ARCHS[@]}"; do
	if [[ $REPO_UPDATE != "skip_updates" || ! -f "repo-$arch.txt" ]]; then
		wget -q $BASE_REPO_URL$arch"/tcz/" -O "repo-$arch.txt"
		if [[ $REPO_UPDATE == "skip_updates" ]]; then
			echo "Repository for $arch missing, forcing online update!"
			REPO_UPDATE=forced_update
		fi
	fi
done

install_package()
{
	pkg=$1
	url=$2
	if [[ -f $PKG_PATH/$pkg ]]; then
		return 0
	fi
	if [[ "$pkg" = "" ]]; then
		return 0
	fi
	echo "Installing package $pkg from $url to $PKG_PATH"

	if [[ $REPO_UPDATE != "skip_updates" ]]; then

		# Download package if not already there
		wget -N -q --show-progress $url
		wget -q "$url.md5.txt" -O "$pkg.md5.txt"
		wget -q "$url.dep" -O "$pkg.dep"
		if [[ ! -f $pkg || ! -f "$pkg.md5.txt" || ! -f "$pkg.dep" ]]; then
			echo "Failed to download $pkg or its hash and dependency list!"
			return 1
		fi
	else
		if [[ ! -f $pkg || ! -f "$pkg.md5.txt" || ! -f "$pkg.dep" ]]; then
			echo "Assumed $pkg, including hash and dependency list, to be downloaded already, but can't find it! Consider removing skip_updates option."
			return 1
		fi
	fi

	# Verify MD5 checksum
	MD5=$(md5sum "$pkg")
	if [[ $(cat "$pkg.md5.txt") != $MD5 ]]; then
		# Delete package and stop script if MD5 sum is wrong
		rm "$pkg"
		DEPENDENCIES=
		echo "Could not install $pkg, MD5 mismatch!"
		return 1
	fi

	# Copy package to TCE folder
	cp "$pkg" $PKG_PATH/
	cp "$pkg.md5.txt" $PKG_PATH/
	cp "$pkg.dep" $PKG_PATH/

	# Record dependencies to be handled in the next iteration
	DEPENDENCIES="$DEPENDENCIES $(cat "$pkg.dep")"
	return 0
}

# Recursively download, verify, and install dependencies
while [[ -n "$DEPENDENCIES" ]]; do
	PACKAGES=$DEPENDENCIES
	DEPENDENCIES=

	for pkgname in $PACKAGES; do

		# Check if already installed
		if [[ -f $PKG_PATH/$pkgname ]]; then
			continue
		fi

		# If package name contains KERNEL keyword, replace with our kernel (or any other)
		if [[ $(grep -c "^$pkgname$" "repo-$MAIN_ARCH.txt") != 1 ]]; then
			for arch in "${ARCHS[@]}"; do
				pkg_k=$(echo $pkgname | sed s/KERNEL/$KERNEL.*/)
				pkg_r=$(grep "$pkg_k$" "repo-$arch.txt")
				if [[ "$pkg_r" = "" ]]; then
					echo "WARNING: Found no $pkg_k package in $arch repository!"
				else
					install_package $pkg_r "$BASE_REPO_URL$arch/tcz/$pkg_r"
					if [[ $? -ne 0 ]]; then
						return 1
					fi
				fi
			done
		else
			install_package $pkgname "$BASE_REPO_URL$MAIN_ARCH/tcz/$pkgname"
			if [[ $? -ne 0 ]]; then
				return 1
			fi
		fi
	done
done

popd > /dev/null

for pkgname in $OVERLAY_PACKAGES; do
	echo "Installing overlay package $pkgname into $PKG_PATH!"
	cp "$pkgname" $PKG_PATH/
	echo $pkgname >> $TCE_PATH/onboot.lst
done

echo "Setup and configured piCore!"

# Unmounting is done by traps when mounting