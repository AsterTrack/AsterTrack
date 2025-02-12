# AsterTrack Controller

Firmware for the CH32V307 in the AsterTrack Controller hardware. <br>
It acts as a middleman between the cameras and the host computer, facilitating communication between them through the serial and USB interfaces, synchronising the cameras and establishing a shared time sync with the host.

## Building

The build system is designed for and tested on Linux.
It may be possible to build on Windows or Mac as well, so feel free to try and document the process, but this isn't supported at the moment.

### Requirements
**GNU RISC-V Embedded Toolchain**: <br>
*Recommended* Use a precompiled xPack version. <a href="https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/tag/v12.4.0-1">v12</a> has been confirmed to work. You may extract it to e.g. `/opt` and update the TOOL_PATH in the Makefile. <br>
The toolchain can also be built manually using <a href="https://github.com/riscv-collab/riscv-gnu-toolchain">this source repository</a> or using <a href="https://crosstool-ng.github.io/">crosstools-ng</a>. <br>
Whatever method you choose, make sure TOOL_PATH in the Makefile points to the folder. <br>

### Compilation
Run task `Build CH32V307 Controller` to build the firmware for the CH32V307. <br>
Run task `Build CH32V307 Configuration Program` to build the configuration firmware that configures the hardware. <br>

### Flashing using the bootloader (recommended)

#### One-Time Setup:
1. Run task `Build CH32V307 Flashing Tool` to build the flashing tool. <br>
2. Ensure the Controller is connected through the Data USB port. <br>
3. Press and hold the `Flash` button on the controller hardware or bridge the `BOOT1` pin. The controller will reboot into the bootloader/ISP mode and should appear as WinChipHead. <br>
4. Run task `Configure CH32V307` to flash the configuration firmware onto the CH32V307. It will automatically reboot and configure itself. <br>

#### Flashing Firmware:
1. Ensure the Controller is connected through the Data USB port. <br>
2. Press and hold the `Flash` button on the controller hardware or bridge the `BOOT1` pin. The controller will reboot into the bootloader/ISP mode and should appear as WinChipHead. <br>
3. Run task `Flash CH32V307` to flash the firmware onto the CH32V307. It will reboot and be operable immediately. <br>

#### Troubleshooting
If you face permission errors on linux, see the section `Linux Permissions` to set them up. <br>
If you have troubles flashing, first try to forcefully enter bootloader mode by disconnecting all cables from the controller, press and hold the `Reset` button (not the `Flash` button) or bridge the `BOOT0` pin, and plug the data USB back in while the `Reset` button is still held down. Then, continue with the flashing steps. <br>
Should you still have troubles, you may try flashing using SWD and OpenOCD, though this still won't work if the bootloader is bricked. <br>

### Flashing using SWD and OpenOCD
This flashing method is complicated, requires extra hardware, and should not ordinarily be used. It is also currently only tested on Linux. <br>
Remember to still first flash the configurator program to configure the chip once - see above for details. <br>
**Hardware Requirements:** An SWD programmer. You can use a simple FTDI programmer with 3.3V output (IMPORTANT!), but you could also use an ST-Link. <br>
**Building OpenOCD:** You need a modified version of OpenOCD for the CH32V307 which is shipped with MounRiver Studio. The source code is sporadically released by WCH on request, some versions of which can be found <a href="https://github.com/Seneral/riscv-openocd-wch">in this repo</a>. A more recent version is not required for the CH32V307. You may put the resulting binary in your toolchain bin folder - just make sure to update the binary path in `.vscode/tasks.json`.<br>
**Hardware Connection:** The SWD programmer needs to be connected to the debug pins of the board. Always refer to the documentation of the exact board you are using. The official AsterTrack controller PCB has the 4 required pins broken out and labelled: `3V`: 3.3V, `IO`: SW Data IO, `CL`: SW Clock, `G`: Ground <br>
**Flashing:** Follow the same steps as above to get the chip into bootloader mode, but use tasks `Configure CH32V307 OpenOCD` and `Flash CH32V307 OpenOCD` instead.

### Without VS Code
Follow the same steps and use the commands found in `.vscode/tasks.json` instead.

### Linux Permissions
To make sure the flashing program (and the AsterTrack Application) has access to the controller usb device:
- For the firmware: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="16c0", ATTR{idProduct}=="05dc", MODE="0660", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/00-usb-permissions.rules`
- For the bootloader: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="4348", ATTR{idProduct}=="55e0", MODE="0660", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/00-usb-permissions.rules`
- `sudo udevadm control --reload-rules`

For this to work, verify that you are already part of the plugdev group:
`getent group | grep plugdev`

If not, make sure such a group exists and join it:
- `sudo groupadd plugdev`
- `sudo usermod -a -G plugdev $USER`
- and then restart (or log out and back in)

## License
The AsterTrack Controller Firmware is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header. <br>
Notably, the usb_driver is licensed under MPL 2.0, as it is a full reimplementation of the USB stack for the CH32V307 with much more robust and fully-featured control transfers than the vendor-provided implementation.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.