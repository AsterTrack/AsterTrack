#!/bin/bash

mkdir -p build-env/videocore
mkdir -p build-env/util
mkdir -p build-env/comm
mkdir -p build-env/mcu
mkdir -p build-env/blob
mkdir -p build-env/camera
mkdir -p build-env/vcsm
mkdir -p build-env/hash
mkdir -p build-env/processing

pushd build-env
../source/version.sh
popd

SHARED="-DARMV7 -Ienv/include -I../shared -Idependencies -Isource -g -rdynamic -fno-omit-frame-pointer -Wall"
CPP="-std=c++20 -Wno-sign-compare -Wno-pointer-arith -Wno-format-overflow -Wno-unused-label -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function -Wno-format -O1"

/usr/bin/gcc $SHARED -std=gnu17 -O1 -o build-env/videocore/mailbox.c.o -c source/videocore/mailbox.c
/usr/bin/gcc $SHARED -std=gnu17 -O1 -o build-env/videocore/vc_base.c.o -c source/videocore/vc_base.c
/usr/bin/gcc $SHARED -std=gnu17 -O1 -o build-env/videocore/qpu_program.c.o -c source/videocore/qpu_program.c
/usr/bin/gcc $SHARED -std=gnu17 -O1 -o build-env/videocore/qpu_info.c.o -c source/videocore/qpu_info.c
/usr/bin/gcc $SHARED -std=gnu17 -O1 -o build-env/videocore/vpu_program.c.o -c source/videocore/vpu_program.c
/usr/bin/gcc $SHARED -std=gnu17 -O1 -o build-env/util/fbUtil.c.o -c source/util/fbUtil.c
/usr/bin/g++ $SHARED $CPP -o build-env/camera/gcs_v4l2.cpp.o -c source/camera/gcs_v4l2.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/vcsm/vcsm.cpp.o -c source/vcsm/vcsm.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/parsing.cpp.o -c source/comm/parsing.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/comm.cpp.o -c source/comm/comm.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/server.cpp.o -c source/comm/server.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/uart.cpp.o -c source/comm/uart.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/wireless.cpp.o -c source/comm/wireless.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/firmware.cpp.o -c source/comm/firmware.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/mcu/mcu.cpp.o -c source/mcu/mcu.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/mcu/stm32_bootloader.cpp.o -c source/mcu/stm32_bootloader.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/blob/detection.cpp.o -c source/blob/detection.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/visualisation.cpp.o -c source/visualisation.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/processing/framesync.cpp.o -c source/processing/framesync.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/processing/masking.cpp.o -c source/processing/masking.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/processing/processing.cpp.o -c source/processing/processing.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/version.cpp.o -c source/version.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/state.cpp.o -c source/state.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/main.cpp.o -c source/main.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/util/crash_handler.cpp.o -c source/util/crash_handler.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/util/image.cpp.o -c ../shared/util/image.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/util/timesync.cpp.o -c ../shared/util/timesync.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/comm/protocol_stream.cpp.o -c ../shared/comm/protocol_stream.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/blob/refinement.cpp.o -c ../shared/blob/refinement.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/blob/blob.cpp.o -c ../shared/blob/blob.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/hash/crc32.cpp.o -c ../shared/hash/crc32.cpp
/usr/bin/g++ $SHARED $CPP -o build-env/hash/sha256.cpp.o -c ../shared/hash/sha256.cpp

rm -r build-env