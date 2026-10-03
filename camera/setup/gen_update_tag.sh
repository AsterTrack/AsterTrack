#!/bin/bash

version=$(cat ../source/version | head -n 1 )
fwDesc=$(cat ../source/version | tail -n +2)

IFS='.' read -r -a ver <<< "$version"

# Clear firmware tag
tag=$1
if [[ -z "$tag" ]]; then
    tag=tag.bin
fi
: > $tag

# Descriptor Texts
printf "%s" "$fwDesc" >> $tag
# Descriptor Lengths (16 bit each)
v=$(printf "%04X" "0")
printf "%b" "\\x${v:2:2}\\x${v:0:2}" >> $tag
v=$(printf "%04X" "${#fwDesc}")
printf "%b" "\\x${v:2:2}\\x${v:0:2}" >> $tag
# Version Number (4 bytes)
v0=$(printf '%x' ${ver[0]})
v1=$(printf '%x' ${ver[1]})
v2=$(printf '%x' ${ver[2]})
v3=$(printf '%x' 0)
printf "%b" "\x$v0\x$v1\x$v2\x$v3" >> $tag
# Tag Header Information (Version 1, Simple Update Package)
printf '\x00\x00\x83\x02' >> $tag
# Tag Key
printf '\x08\x39\xA5\x73\x20\x72\xAC\x80' >> $tag
