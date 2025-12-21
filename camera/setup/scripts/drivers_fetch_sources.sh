#!/bin/sh

rm -rf /mnt/mmcblk0p4/kernel/module
mkdir -p /mnt/mmcblk0p4/kernel/module
cd /mnt/mmcblk0p4/kernel/module

wget -q https://raw.githubusercontent.com/raspberrypi/linux/refs/heads/rpi-6.12.y/drivers/media/i2c/ov9282.c
#wget -q https://raw.githubusercontent.com/raspberrypi/linux/refs/heads/rpi-6.1.y/drivers/media/i2c/ov9281.c

# Patch newer ov9282 driver to always use continuous clock
sed -i -z 's/ov9282->noncontinuous_clock =\n\s*bus_cfg.bus.mipi_csi2.flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;/ov9282->noncontinuous_clock = false;/g' ov9282.c

# Patch older ov9281 driver for newer V4L2 API
#sed -i 's/v4l2_subdev_get_try_format(sd, /v4l2_subdev_state_get_format(/g' ov9281.c
#sed -i 's/v4l2_subdev_get_try_crop(&ov9281->subdev, /v4l2_subdev_state_get_crop(/g' ov9281.c
#sed -i -z 's/ov9281_probe(struct i2c_client \*client,\n\s*const struct i2c_device_id \*id)/ov9281_probe(struct i2c_client \*client)/g' ov9281.c

#echo "obj-m := ov9281.o ov9282.o" > Makefile
echo "obj-m := ov9282.o" > Makefile

cd /home/tc