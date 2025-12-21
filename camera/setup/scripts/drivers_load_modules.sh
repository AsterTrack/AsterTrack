#!/bin/sh

DRIVER_SRC="/home/tc/drivers"
DRIVER_TGT="/lib/modules/$(uname -r)/kernel/drivers/media/i2c"
CUSTOM_DRIVERS="ov9282.ko"
LOAD_DRIVERS="ov9282.ko"

for DRIVER in $CUSTOM_DRIVERS; do

	if [[ ! -f "$DRIVER_SRC/$DRIVER" ]]; then
		echo "Driver $DRIVER has not been built!"
		continue
	fi

	# Move drivers into appropriate directory
	mkdir -p $DRIVER_TGT
	cp "$DRIVER_SRC/$DRIVER" $DRIVER_TGT

done

echo "Registering drivers..."

# Update modules.dep to replace/add drivers
depmod $(uname -r)

echo "Loading drivers..."

for DRIVER in $LOAD_DRIVERS; do
	modprobe $DRIVER
done


