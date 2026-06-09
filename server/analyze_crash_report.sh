#!/usr/bin/env bash

CRASH_REP="$1"
BIN_PATH="build/"

calc_base_address() {
    MAP=$1
    echo "Mapped Section: $MAP" >&2
    local LOAD_OFFSET=$(echo $MAP | cut -f1 -d"-")
    local FILE_OFFSET=$(echo $MAP | awk -v col=3 '{print $col}')
    echo "Loaded $LOAD_OFFSET with file offset $FILE_OFFSET" >&2
    local LOAD_BASE=$(( (16#$LOAD_OFFSET) - (16#$FILE_OFFSET) ))
    local ADDR_BASE=$(printf '%x' $LOAD_BASE)
    echo "    virtual base address of $ADDR_BASE" >&2
    echo $ADDR_BASE
}

calc_section_address() {
    MAP=$1
    SECT=$2
    echo "Mapped Section: $MAP" >&2
    echo "Binary Section: $SECT" >&2
    local LOAD_OFFSET=$(echo $MAP | cut -f1 -d"-")
    local FILE_OFFSET=$(echo $MAP | awk -v col=3 '{print $col}')
    local SECT_OFFSET=$(echo $SECT | awk -v col=6 '{print $col}')
    echo "Loaded $LOAD_OFFSET with file offset $FILE_OFFSET, segment has file offset $SECT_OFFSET" >&2
    local MISALIGN=$(( (16#$SECT_OFFSET) - (16#$FILE_OFFSET) ))
    local LOAD_SECT=$(( (16#$LOAD_OFFSET) + $MISALIGN ))
    local ADDR_SECT=$(printf '%x' $LOAD_SECT)
    echo "    misaligned by $MISALIGN, loaded address of segment $ADDR_SECT" >&2
    echo $ADDR_SECT
}

# Load .text section at right address
# TODO: Ensure other sections are also properlly mapped
# This is a bit more complex as multiple sections are mapped as one
# And the controls for that in LLDB are lacking

if [[ ! -z "$(grep astertrack-static $CRASH_REP)" ]]; then

    STATIC_MAP_TEXT=$(cat $CRASH_REP | grep -A100000 "Memory map:" | grep "astertrack-static" | grep "r-xp")
    STATIC_SECTION_TEXT=$(objdump --section-headers --section=".text" "$BIN_PATH/astertrack-static" | grep ".text")

    #STATIC_ADDR="--slide "$(calc_base_address "$STATIC_MAP_TEXT")
    STATIC_ADDR=".text 0x"$(calc_section_address "$STATIC_MAP_TEXT" "$STATIC_SECTION_TEXT")

    echo "" > crash_inspect.lldb
    echo "target create $BIN_PATH/astertrack-static" >> crash_inspect.lldb
    echo "target modules load --file astertrack-static $STATIC_ADDR" >> crash_inspect.lldb

else

    SERVER_MAP_TEXT=$(cat $CRASH_REP | grep -A100000 "Memory map:" | grep "astertrack-server" | grep "r-xp")
    SERVER_SECTION_TEXT=$(objdump --section-headers --section=".text" "$BIN_PATH/astertrack-server" | grep ".text")
    INTERFACE_MAP_TEXT=$(cat $CRASH_REP | grep -A100000 "Memory map:" | grep "astertrack-interface.so" | grep "r-xp")
    INTERFACE_SECTION_TEXT=$(objdump --section-headers --section=".text" "$BIN_PATH/astertrack-interface.so" | grep ".text")

    #SERVER_ADDR="--slide "$(calc_base_address "$SERVER_MAP_TEXT")
    #INTERFACE_ADDR="--slide "$(calc_base_address "$INTERFACE_MAP_TEXT")
    SERVER_ADDR=".text 0x"$(calc_section_address "$SERVER_MAP_TEXT" "$SERVER_SECTION_TEXT")
    INTERFACE_ADDR=".text 0x"$(calc_section_address "$INTERFACE_MAP_TEXT" "$INTERFACE_SECTION_TEXT")

    echo "" > crash_inspect.lldb
    echo "target create $BIN_PATH/astertrack-server" >> crash_inspect.lldb
    echo "target modules load --file astertrack-server $SERVER_ADDR" >> crash_inspect.lldb
    echo "target modules add $BIN_PATH/astertrack-interface.so" >> crash_inspect.lldb
    echo "target modules load --file astertrack-interface.so $INTERFACE_ADDR" >> crash_inspect.lldb
fi

STACK_TRACE=$(cat $CRASH_REP | grep "^\[[0-9]*\]:\s0x" | sed -E 's/^\[[0-9]+\]:\s0x([0-9A-F]*)\s.*\/([^(]+)\(.*/\1\/\2/')

echo "============="
echo "Adjusted StackTrace (ADDR - 1):"
for STACK in $STACK_TRACE; do
    ADDR=$(echo $STACK | cut -f1 -d"/")
    IMAGE=$(echo $STACK | cut -f2 -d"/")
    PREV_ADDR=$(printf '%x' $(( (16#$ADDR) - 1 )))
    echo "image lookup -a $PREV_ADDR (from $IMAGE)"
done
echo "============="

lldb -s crash_inspect.lldb