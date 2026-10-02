#!/bin/bash

version=$(cat ../source/version | head -n 1 )
fwDesc=$(cat ../source/version | tail -n +2)
fwCompat=$(uname -r | grep -o -e '.*-piCore' || uname -r)
# Remove architecture, each version already compiles for all supported architectures

IFS='.' read -r -a ver <<< "$version"
build=$(($RANDOM % 255))

echo -e \
"#define FIRMWARE_DESCRIPTOR \"$fwDesc\"\\n"\
"#define FIRMWARE_COMPATIBILITY \"$fwCompat\"\\n"\
"#define FW_MAJOR ${ver[0]}\\n"\
"#define FW_MINOR ${ver[1]}\\n"\
"#define FW_PATCH ${ver[2]}\\n"\
"#define FW_BUILD ${build}" > ../source/version.h

# Clear firmware tag
tag=tag.bin
: > $tag

# Descriptor Texts
printf "%s" "$fwCompat" >> $tag
printf "%s" "$fwDesc" >> $tag
# Descriptor Lengths (16 bit each)
v=$(printf "%04X" "${#fwCompat}")
printf "%b" "\\x${v:2:2}\\x${v:0:2}" >> $tag
v=$(printf "%04X" "${#fwDesc}")
printf "%b" "\\x${v:2:2}\\x${v:0:2}" >> $tag
# Version Number (4 bytes)
v0=$(printf '%x' ${ver[0]})
v1=$(printf '%x' ${ver[1]})
v2=$(printf '%x' ${ver[2]})
v3=$(printf '%x' ${build})
printf "%b" "\x$v0\x$v1\x$v2\x$v3" >> $tag
# Tag Header Information (Version 2, Camera SBC Package)
printf '\x00\x00\x17\x02' >> $tag
# Tag Key
printf '\x08\x39\xA5\x73\x20\x72\xAC\x80' >> $tag
