if [[ "$EUID" != 0 ]]; then
	echo "Need sudo to execute!"
	exit 1
fi

source config.txt

# Parameters
DEVICE_PATH=$1

if [[ -z "$DEVICE_PATH" ]]; then
	echo "Please specify the device path as the first argument!"
	exit 1
fi
# Make sure kernel knows about the partitions to we can address them with p subscript
partprobe $DEVICE_PATH
PARTITION_DATA=$DEVICE_PATH"p2"
if [[ ! -b "$PARTITION_DATA" ]]; then
	PARTITION_DATA=$DEVICE_PATH"2"
	if [[ ! -b "$PARTITION_DATA" ]]; then
		echo "Cannot find second partition of device $DEVICE_PATH!"
		exit 1
	fi
fi

if [[ ! -b "$DEVICE_PATH" ]]; then
	echo "Device $DEVICE_PATH does not exist!"
	exit 1
fi

if [[ ! -d $DATA_PATH ]]; then
	echo "Invalid data path $DATA_PATH to write to $DATA_FILE!"
	exit 1
fi

echo "Mounting TCE partition..."

# Mount data partition
if [[ -d $MOUNT_DATA ]]; then
	umount -q -f $MOUNT_DATA
	rmdir $MOUNT_DATA
fi
mkdir $MOUNT_DATA
mount $PARTITION_DATA $MOUNT_DATA

# Make sure it is unmounted even when script is interrupted
trap "umount -f $PARTITION_DATA && echo 'Unmounting TCE partition...'" SIGINT EXIT
