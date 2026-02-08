# Newest version from ARM
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# For Windows with MSYS2/Cygwin: Change all backslashes \ to forward slashes / in your path!
TOOL_PATH=/opt/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi
TOOL_PREFIX = $(TOOL_PATH)/bin/arm-none-eabi-
TOOL_INCLUDE = -I$(TOOL_PATH)/arm-none-eabi/include -I$(TOOL_PATH)/lib/gcc/arm-none-eabi/*/include