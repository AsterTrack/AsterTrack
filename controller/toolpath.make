# Newest version from xPack
# https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases
# For Windows with MSYS2/Cygwin: Change all backslashes \ to forward slashes / in your path!
TOOL_PATH=/opt/xpack-riscv-none-elf-gcc-15.2.0-1
TOOL_PREFIX = $(TOOL_PATH)/bin/riscv-none-elf-
TOOL_INCLUDE = -I$(TOOL_PATH)/riscv-none-elf/include -I$(TOOL_PATH)/lib/gcc/riscv-none-elf/*/include