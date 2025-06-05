
#######################################
# BUILD OPTIONS
#######################################

TARGET = TrackingCameraMCU
# Allow for debugging and flashing over JTAG/SWD - only disable if you know you can flash over other means!
# Worst case, you may still be able to use SWD if booting with Boot0 active
DEBUG_FLASH = 1
# Optimisation level, O0 for debugging, O3 for release
OPT = -O1

# Enables logging at different levels
LOG_FLAGS = -DENABLE_LOG
# Generous Debug Logging. Don't use at high framerates
#LOG_FLAGS += -DDEBUG_LOG
# Additional Information (Incidents that could give context to problems)
#LOG_FLAGS += -DINFO_LOG
# Warnings (Could indicate problems)
LOG_FLAGS += -DWARN_LOG
# Genuine Errors (Loss of data, etc)
LOG_FLAGS += -DERR_LOG
# Known Errors (presumably hardware errors in USB PHY)
#LOG_FLAGS += -DKERR_LOG
# Timing Errors (Taking unreasonably long for an operation)
#LOG_FLAGS += -DTERR_LOG
# Temporary log during debugging
#LOG_FLAGS += -DTEMP_LOG
# COMMunication Log - needs WARN_LOG as well
LOG_FLAGS += -DCOMM_LOG
# Command Debug Log
LOG_FLAGS += -DCMDD_LOG
# UART Hardware Debug Log
#LOG_FLAGS += -DUART_LOG

#######################################
# BUILD FILES
#######################################

BUILD_DIR = build

# ----- C sources -----

C_SOURCES_ARM_LL = \
source/main.c \
source/uartd.c \
targets/ARM/source/config_arm_ll.c \
targets/ARM/source/uart_arm_ll.c

C_SOURCES_STM32G030 = $(C_SOURCES_ARM_LL) \
targets/ARM/source/system_stm32g0xx.c

C_SOURCES = $(C_SOURCES_$(DEVICE))

C_SOURCES_SHARED = \
mcu/util.c

# ----- C includes -----

C_INCLUDES_STM32G030 = \
-Itargets/ARM/source \
-Itargets/ARM/drivers/CMSIS/Include \
-Itargets/ARM/drivers/CMSIS/Device/ST \
-Itargets/ARM/drivers/CMSIS/Device/ST/STM32G0xx/Include \
-Itargets/ARM/drivers/STM32G0xx_HAL_Driver/Inc

C_INCLUDES = \
-Isource \
-I../shared \
-I../shared/mcu \
$(C_INCLUDES_$(DEVICE))

# ----- C defines -----

C_DEFS_STM32G030 = \
-DSTM32 \
-DARM_LL \
-DSTM32G0 \
-DSTM32G030xx \
-DBOARD_OLD \
-DUSE_UART

C_DEFS = $(C_DEFS_$(DEVICE))

# ----- ASM sources -----

ASM_SOURCES_STM32G030 = targets/ARM/startup_STM32G030xx.s
ASM_SOURCES = $(ASM_SOURCES_$(DEVICE))

# ----- Link script -----

LDSCRIPT_STM32G030 = targets/ARM/link_STM32G030K6Tx.ld
LDSCRIPT = $(LDSCRIPT_$(DEVICE))


#######################################
# BUILD TOOLS
#######################################

TOOL_PATH=/opt/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin/
#TOOL_PATH=/opt/gcc-arm-none-eabi-10.3-2021.10/bin/
TOOL_PREFIX = $(TOOL_PATH)arm-none-eabi-

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

ifeq ($(DEVICE), STM32G030)
	MCU = -mcpu=cortex-m0plus -mthumb
else
	MCU =
endif

COMMON_FLAGS = -Wall -fdata-sections -ffunction-sections -funsigned-char
#COMMON_FLAGS += -flto=auto

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
LDFLAGS = $(COPTIONS) \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
	-Wl,--gc-sections \
	-specs=nano.specs -specs=nosys.specs \
	-lc -lm -lnosys

#######################################
# BUILD TARGETS
#######################################

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).asm

C_OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o)) $(addprefix $(BUILD_DIR)/shared/,$(C_SOURCES_SHARED:.c=.o))
OBJECTS = $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o)) $(C_OBJECTS)

$(shell mkdir -p $(dir $(OBJECTS)) >/dev/null)

$(BUILD_DIR)/shared/%.o: ../shared/%.c Makefile
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/shared/$(<:.c=.lst) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.c=.lst) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(LD) $(OBJECTS) $(LDFLAGS) -T$(LDSCRIPT) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

$(BUILD_DIR)/%.asm: $(BUILD_DIR)/%.elf
	$(DP) -d $< > $@


#######################################
# CLEAN UP
#######################################

clean:
	-rm -fR $(BUILD_DIR)


#######################################
# DEPENDENCIES
#######################################

-include $(C_OBJECTS:.o=.d)
