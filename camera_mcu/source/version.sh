#!/bin/sh

version=$(cat source/version | head -n 1 )
desc=$(cat source/version | tail -n +2)
IFS='.' read -r -a ver <<< "$version"

build=$(($RANDOM % 255))

echo -e \
"#define FIRMWARE_DESCRIPTOR \"$desc\"\\n"\
"#define FW_MAJOR ${ver[0]}\\n"\
"#define FW_MINOR ${ver[1]}\\n"\
"#define FW_PATCH ${ver[2]}\\n"\
"#define FW_BUILD ${build}" > build/version.h

# Clear firmware tag
tag=build/tag.bin
: > $tag

# Descriptor Text
printf "%s" "$desc" >> $tag
# Descriptor Length (16 bit)
v=$(printf "%04X" "${#desc}")
printf "%b" "\\x${v:2:2}\\x${v:0:2}" >> $tag
# Version Number (4 bytes)
v0=$(printf '%x' ${ver[0]})
v1=$(printf '%x' ${ver[1]})
v2=$(printf '%x' ${ver[2]})
v3=$(printf '%x' ${build})
printf "%b" "\x$v0\x$v1\x$v2\x$v3" >> $tag
# Tag Header Information (Version 1, Camera MCU FW)
printf '\x00\x00\x9F\x01' >> $tag
# Tag Key
printf '\x08\x39\xA5\x73\x20\x72\xAC\x80' >> $tag