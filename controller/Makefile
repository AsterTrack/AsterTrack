
#######################################
# BUILD OPTIONS
#######################################

TARGET = TrackingController
# Allow for debugging and flashing over JTAG/SWD - only disable if you know you can flash over other means!
# Worst case, you may still be able to use SWD if booting with Boot0 active
DEBUG_FLASH = 1
# Optimisation level, O0 for debugging, O3 for release
OPT = -O3
LOG_FLAGS =

# Enables logging at different levels
LOG_FLAGS += -DENABLE_LOG
LOG_FLAGS += -DDEBUG_USE_INT
LOG_FLAGS += -DENABLE_EVENTS
LOG_FLAGS += -DEVENT_USE_INT
# Generous Debug Logging. Don't use at high framerates
#LOG_FLAGS += -DDEBUG_LOG
# Additional Information (Incidents that could give context to problems)
#LOG_FLAGS += -DINFO_LOG
# Warnings (Could indicate problems)
LOG_FLAGS += -DWARN_LOG
# Genuine Errors (Loss of data, etc)
LOG_FLAGS += -DERR_LOG
# Known Errors (presumably hardware errors in USB PHY)
LOG_FLAGS += -DKERR_LOG
# Timing Errors (Taking unreasonably long for an operation)
#LOG_FLAGS += -DTERR_LOG
# Temporary log during debugging
#LOG_FLAGS += -DTEMP_LOG
# COMMunication Log - needs WARN_LOG as well
LOG_FLAGS += -DCOMM_LOG
# Command Debug Log
LOG_FLAGS += -DCMDD_LOG
# USB Hardware Debug Log
#LOG_FLAGS += -DUSBD_LOG
# USB PD Debug Log
#LOG_FLAGS += -DUSBPD_LOG
#LOG_FLAGS += -DPD_CAPS
# UART Hardware Debug Log
#LOG_FLAGS += -DUART_LOG
# UART Hardware Debug Traces
#LOG_FLAGS += -DUART_TRACE
# Start Of Frame Debug Log
#LOG_FLAGS += -DSOFD_LOG
# Overflow Log / Dataflow focused on overflow usage
#LOG_FLAGS += -DOVFL_LOG
# Blockflow Log / Dataflow focused on blocks and where they go
#LOG_FLAGS += -DBLKF_LOG

#######################################
# BUILD FILES
#######################################

BUILD_DIR = build

# ----- C sources -----

C_SOURCES = \
source/main.c \
source/util.c \
source/usbd.c \
source/uartd.c \
source/config.c \
source/usb_driver.c \
source/uart_driver.c \
source/pd_driver.c \
dependencies/PDlib/src/fusb302b.c \
dependencies/PDlib/src/policy_engine_states.c \
dependencies/PDlib/src/policy_engine.c \
ch32v307/src/system_ch32v30x.c

# ----- C includes -----

C_INCLUDES = \
-Isource \
-Idependencies/libusb \
-Idependencies/PDlib/include \
-Idependencies/PDlib/src \
-Ich32v307/drivers \
-Ich32v307/drivers/ch32v30x/inc \
-I../shared

# ----- C defines -----

C_DEFS = \
-DCH32 \
-DCH32V3 \
-DCH32V307

# ----- ASM sources -----

ASM_SOURCES = ch32v307/startup_CH32V30x_D8C.s

# ----- Link script -----

LDSCRIPT = ch32v307/link_CH32V307VCTx.ld
LDSCRIPT_Config = ch32v307/link_CH32V307VCTx_config.ld


#######################################
# BUILD TOOLS
#######################################

# Newest version from xPack
# https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/tag/v12.2.0-1
#-march=rv32imac_zicsr
TOOL_PATH=/opt/riscv-none-elf-gcc-xpack-12-2/bin/
TOOL_PREFIX = $(TOOL_PATH)riscv-none-elf-

# Older 8.2.0 version from xPack
# https://github.com/xpack-dev-tools/riscv-none-embed-gcc-xpack/releases/tag/v8.2.0-3.1
# This xPack version is used as the basis for MounRiver Studio, with modifications (at least custom interrupts code)
# But cortex-debug extension requires gdb version >= 9 so can't be used to debug
#-march=rv32imac
#TOOL_PATH=/opt/riscv-none-elf-gcc-xpack-8-2/bin/
#TOOL_PREFIX = $(TOOL_PATH)riscv-none-embed-

CC = $(TOOL_PREFIX)gcc
LD = $(CC)
AS = $(TOOL_PREFIX)gcc -x assembler-with-cpp
CP = $(TOOL_PREFIX)objcopy
DP = $(TOOL_PREFIX)objdump
SZ = $(TOOL_PREFIX)size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S


#######################################
# BUILD FLAGS
#######################################

MCU = -march=rv32imac_zicsr -mabi=ilp32
# Make sure to match march with the toolchain selected above!

COMMON_FLAGS = -Wall -fdata-sections -ffunction-sections -funsigned-char
COMMON_FLAGS += -msmall-data-limit=8 -mno-save-restore -fmessage-length=0 -flto=auto

# Add debug options
ifeq ($(DEBUG_FLASH), 1)
	DFLAGS += -g -gdwarf-2 -DDEBUG -DDEBUG_FLASH
endif
DFLAGS += $(LOG_FLAGS)


# Generate files listing source dependencies
DEPEND_FLAGS = -MMD -MP -MF"$(@:%.o=%.d)"

# Compiler flags
ASFLAGS = $(MCU) $(OPT) $(COMMON_FLAGS)
COPTIONS = $(MCU) $(DFLAGS) $(OPT) $(COMMON_FLAGS)
CFLAGS = $(COPTIONS) $(C_DEFS) $(C_INCLUDES) $(DEPEND_FLAGS) -specs=nano.specs 

# Linker flags
LDFLAGS = $(COPTIONS) -T$(LDSCRIPT) \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
	-Wl,--gc-sections \
	-specs=nano.specs -specs=nosys.specs \
	-nostartfiles 

#######################################
# BUILD TARGETS
#######################################

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

configure: $(BUILD_DIR)/configurator.bin

flash: $(BUILD_DIR)/isp55e0

$(BUILD_DIR)/isp55e0: tools/isp55e0.c
	gcc tools/isp55e0.c -O2 -Wall -Werror -lusb-1.0 -o $@

$(BUILD_DIR)/configurator.elf: tools/configure_ch32v307.c $(BUILD_DIR)/ch32v307/startup_CH32V30x_D8C.o Makefile
	$(CC) tools/configure_ch32v307.c $(BUILD_DIR)/ch32v307/startup_CH32V30x_D8C.o $(COPTIONS) $(C_DEFS) $(C_INCLUDES) $(DEPEND_FLAGS) $(LDFLAGS) -o $@
	$(SZ) $@
# CFLAGS already defines nano.specs passing $(COPTIONS) $(C_DEFS) $(C_INCLUDES) $(DEPEND_FLAGS) manually
$(BUILD_DIR)/configurator.bin: $(BUILD_DIR)/configurator.elf | $(BUILD_DIR)
	$(BIN) $< $@

C_OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS = $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o)) $(C_OBJECTS)

$(shell mkdir -p $(dir $(OBJECTS)) >/dev/null)

$(BUILD_DIR)/%.o: %.c Makefile
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@


#######################################
# CLEAN UP
#######################################

clean:
	-rm -fR $(BUILD_DIR)


#######################################
# DEPENDENCIES
#######################################

-include $(C_OBJECTS:.o=.d)
