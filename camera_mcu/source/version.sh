#!/bin/bash

rev=$1
echo "Got default $rev"

# Ensure complete rebuild when board revision changes
# tasks.json may have already handled this if we use bear, otherwise, do here
if [[ -f "build/rev" ]]; then
	if [[ -z $rev ]]; then
		rev=$(cat build/rev)
	fi
elif [[ -z $rev ]]; then
	echo "No default board!"
	rev="REV_1.2"
	echo "Selecting default board as $rev"
fi
# Update with current version
mkdir -p "build/$rev"
echo "$rev" > build/rev

version=$(cat source/version | head -n 1 )
fwDesc=$(cat source/version | tail -n +2)
fwCompat=$(cat build/rev)

IFS='.' read -r -a ver <<< "$version"
compat=$(echo $fwCompat | sed -e 's/\./_/g') # Replace . with _ for macro usage
build=$(($RANDOM % 255))

echo -e \
"#define FIRMWARE_DESCRIPTOR \"$fwDesc\"\\n"\
"#define FIRMWARE_COMPATIBILITY \"$fwCompat\"\\n"\
"#define BOARD_REV $compat\\n"\
"#define FW_MAJOR ${ver[0]}\\n"\
"#define FW_MINOR ${ver[1]}\\n"\
"#define FW_PATCH ${ver[2]}\\n"\
"#define FW_BUILD ${build}\\n" > build/version.h

# Clear firmware tag
tag=build/$rev/tag.bin
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
# Tag Header Information (Version 2, Camera MCU FW)
printf '\x00\x00\x9F\x02' >> $tag
# Tag Key
printf '\x08\x39\xA5\x73\x20\x72\xAC\x80' >> $tag
