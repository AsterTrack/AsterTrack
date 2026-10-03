#!/bin/bash

cp -R ../../camera_mcu/build/TrackingCameraMCU*.bin storage/

pushd storage
rm CameraUpdate.zip
zip CameraUpdate.zip TrackingCameraMCU_*.bin mydata*.tgz
popd

./gen_update_tag.sh storage/tag.bin

cat storage/tag.bin >> storage/CameraUpdate.zip