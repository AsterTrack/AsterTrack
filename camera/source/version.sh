#!/bin/sh

version=$(cat ../source/version | head -n 1 )
desc=$(cat ../source/version | tail -n +2)

build=$(($RANDOM % 255))

echo -e \
"#define FIRMWARE_DESCRIPTOR \"$desc\"\\n"\
"#define FW_MAJOR $(echo $version | cut -d '.' -f "1")\\n"\
"#define FW_MINOR $(echo $version | cut -d '.' -f "2")\\n"\
"#define FW_PATCH $(echo $version | cut -d '.' -f "3")\\n"\
"#define FW_BUILD ${build}" > version.h

# Clear firmware tag
tag=tag.bin
: > $tag

# Descriptor Text
printf "%s" "$desc" >> $tag
# Descriptor Length (16 bit)
v=$(printf "%04X" "${#desc}")
printf "%b" "\\x${v:2:2}\\x${v:0:2}" >> $tag
# Version Number (4 bytes)
v0=$(printf '%x' $(echo $version | cut -d '.' -f "1"))
v1=$(printf '%x' $(echo $version | cut -d '.' -f "2"))
v2=$(printf '%x' $(echo $version | cut -d '.' -f "3"))
v3=$(printf '%x' ${build})
printf "%b" "\x$v0\x$v1\x$v2\x$v3" >> $tag
# Tag Header Information (Version 1, Camera SBC Package)
printf '\x00\x00\x17\x01' >> $tag
# Tag Key
printf '\x08\x39\xA5\x73\x20\x72\xAC\x80' >> $tag