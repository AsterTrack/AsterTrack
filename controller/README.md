# AsterTrack Controller

Firmware for the CH32V307 in the AsterTrack Controller hardware. <br>
It acts as a middleman between the cameras and the host computer, facilitating communication between them through the serial and USB interfaces, synchronising the cameras and establishing a shared time sync with the host.

## Building

The build system is designed for and tested primarily on Linux, but you can compile it on Windows as well.

### Requirements

#### Windows: WSL / MSYS2 / Cygwin
While the toolchain itself is available for Windows, the build system uses GNU Make, and thus you need to install either MSYS2 or Cygwin, or you need to use Windows Subsystem for Linux (WSL) and follow the Linux commands. <br>
While MSYS2 and Cygwin both work fine to compile, some other development and debugging tools have better support on Linux, so WSL is better suited for that. <br>
If you choose to use MSYS2 or Cygwin (both of them are equally good options), you only need to install `make` in either of them, and update the paths in `.vscode/tasks.json` to their respective `bash.exe`. <br>
You can install `make` in MSYS2 by opening the MSYS2 terminal and running `pacman -S make`. <br>
You can install `make` in Cygwin by selecting the `make` packet in the provided setup installer (you may have to set the filter to 'ALL' and then search for 'make').

**GNU RISC-V Embedded Toolchain**: <br>
Download the `risc-none-elf` toolchain for your platform from the <a href="https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases">xPack release</a>. v12 and v15 have been confirmed to work. <br>
The toolchain can also be built manually using <a href="https://github.com/riscv-collab/riscv-gnu-toolchain">this source repository</a> or using <a href="https://crosstool-ng.github.io/">crosstools-ng</a>. <br>
You may place the toolchain in e.g. `/opt` or `C:/` and update the TOOL_PATH in `toolpath.make`.

### Compilation
Run task `Build CH32V307 Controller` to build the firmware for the CH32V307. <br>
You may run `Build CH32V307 Controller (clangd)` on Linux instead if you have `bear` installed and use the clangd LSP. This will update the `compile_commands.json` file required to give clangd an accurate understanding of the project. You may need to delete the build folder first to force a full-rebuild if you used the normal build task before.

## Flashing

On Windows, compilation and flashing using either the built-in flashing tool of AsterTrack, or using wlink, works and is supported.
Debugging might require the use of WSL or installation of MounRiver Studio.

### Automatic Flashing using AsterTrack Server Software (recommended)

This method uses the built-in utility of the Server Software to flash via the CH32s bootloader. No extra hardware is required. This automatically configures the chip correctly. <br>
1. Open the Devices View, select the firmware file under "Controller Firmware Update". <br>
2. Ensure the Controller is connected through the Data USB port. <br>
2. Press and hold the middle `Flash` button on the controller for half a second. The controller will reboot into the bootloader/ISP mode and should appear as WinChipHead. <br>
4. Hit the "Flash Controller in Bootloader" button to flash the firmware onto the CH32V307. It will reboot and be operable immediately. <br>

**Prerequisite on Windows:** The bootloader does not by itself support Windows. You need to use <a href="https://zadig.akeo.ie/">Zadig</a> to assign the WinUSB driver to the USB device "WinChipHead". Further instructions can be found in the Server Interface.

#### Troubleshooting
If you face permission errors on Linux, see the section `Linux Permissions` to set them up. <br>
If you have troubles detecting the WinChipHead bootloader, first try to forcefully enter bootloader mode by disconnecting all cables from the controller, press and hold the middle `Flash` button (or alternatively pull the `BOOT0` pin high), and plug the data USB back in while the `Flash` button is still held down. With this, the controller should reboot into the bootloader/ISP mode and appear as WinChipHead even if the prior firmware was bricked. Then, continue with the flashing steps. <br>
Should you still have troubles, you may try flashing using a WCH-Link(E) Programmer, though this won't work either if the bootloader is bricked or the PCB is broken. <br>

### Flashing using the isp55e0 tool (Linux)
This method uses the isp55e0 tool to flash via the CH32s bootloader. No extra hardware is required, but you need to restart the Controller into bootloader manually before every flash. <br>
Note: This tool is included in the repository and is built on demand. It has been modified from the original version to support the CH32V307 and to automatically configure the chip properly. <br>
1. Ensure the Controller is connected through the Data USB port. <br>
2. Press and hold the middle `Flash` button on the controller for half a second. The controller will reboot into the bootloader/ISP mode and should appear as WinChipHead. <br>
3. Run task `Flash CH32V307 (isp55e0)` to flash the firmware onto the CH32V307. It will reboot and be operable immediately. <br>

### Flashing & Logging using wlink - WCH-LinkE/WCH-Link
This method requires a WCH-Link(E) to flash the MCU. If you have a newer WCH-LinkE, it will connect to the MCU using SDI, and you can use logging over SDI (experimental). Older WCH-Link (without E) connect to the MCU using SWD/JTAG, and only flashing is supported. <br>
Build/Install <a href="https://github.com/ch32-rs/wlink">ws-link</a> so that `wlink` is in the Path. <br>
This method needs a separate firmware to be flashed once to configure the chip correctly if it hasn't been configured already with one of the above flashing methods (and the chip was never flashed before). <br>
Use the `Configure CH32V307 (wlink)` and `Flash CH32V307 (wlink)` tasks, they will work without explicitly restarting the controller into bootloader. <br>
If you have a WCH-LinkE, you may use the task `Flash SDI-Log CH32V307` to flash and start logging via SDI. This is experimental and requires you to delete the build to ensure everything is recompiled with SDI enabled.

**Prerequisite on Windows:** Since WCH-Links by themselves do not support Windows, you need to use <a href="https://zadig.akeo.ie/">Zadig</a> to assign the WinUSB driver to the USB device "WCH-Link (Interface 0)". If you want to use the experimental SDI logging, you also need to assign the "USB Serial (CDC)" driver to "WCH-Link (Interface 1)".

### Flashing & Debugging using OpenOCD - WCH-LinkE/WCH-Link (Linux)
This method uses OpenOCD to flash and requires extra hardware - but it also allows for debugging. It is currently only tested on Linux. <br>
You need either a WCH-LinkE or an older WCH-Link. Other 3.3V SWD programmers likely will not work and are not supported. <br>
This method needs a separate firmware to be flashed once to configure the chip correctly if it hasn't been configured already with one of the above flashing methods (and the chip was never flashed before). <br>
<br>
**Getting OpenOCD:** Both WCH-LinkE and WCH-Link need modified versions of OpenOCD. These are shipped with MounRiver Studio, but the source code is sporadically released by WCH on request, some versions of which can be found <a href="https://github.com/Seneral/riscv-openocd-wch">in this repo</a>. Notably, there are two branches: WCH-Link uses SWD and is older (<a href="https://github.com/Seneral/riscv-openocd-wch/tree/wch-link-swd">wch-link-swd</a>). WCH-LinkE uses SDI and is based on a newer OpenOCD version (<a href="https://github.com/Seneral/riscv-openocd-wch/tree/wch-linke-sdi">wch-linke-sdi</a>). <br>
After building, you may put the resulting binary in `/opt/riscv-openocd-wch-swd/bin` or `/opt/riscv-openocd-wch-sdi/bin`, depending on the version, to match existing paths. <br>
<br>
**Hardware Connection:** The chosen programmer needs to be connected to the debug pins of the board. The official AsterTrack Controller PCB has the 4 pins broken out and labelled: `3V`: 3.3V, `IO`: SW Data IO, `CL`: SW Clock, `G`: Ground. Do NOT connect the 3V pin if you connect the USB Data to a PC - only use it if there is nothing else connected and you want to power the chip from the programmer. <br>
<br>
**Flashing:** Use the `Configure CH32V307 (OpenOCD, WCH-Link*)` and `Flash CH32V307 (OpenOCD, WCH-Link*)` tasks, they will work without explicitly restarting the controller into bootloader. <br>
<br>
**Debugging:** Install the `marus25.cortex-debug` extension and select the relevant debug configuration for either WCH-Link or WCH-LinKE. <br>
Update `armToolchainPath` of the relevant configuration in `.vscode/launch.json` to point to your RISC-V toolchain (this works even though it expects an ARM toolchain). <br>
Now you should be able to debug, set a maximum of 5 breakpoints, and step through code. <br>
<br>
**Windows:** You can try to use MounRiver studio, it should have the required OpenOCD version built-in (at least for WCH-LinkE).
Or you can use WSL and follow the Linux instructions (recommended).
Otherwise, you would have to find a way to compile the custom OpenOCD version using either xPack or MSYS2/Cygwin. This has not been attempted yet.
Then, set `serverpath` in `.vscode/launch.json` to the build folder. Similarly, update the command of the `Flash (OpenOCD)` task in `.vscode/tasks.json` to point to the openocd binary.
Finally, since WCH-Links by themselves do not support Windows, you need to use <a href="https://zadig.akeo.ie/">Zadig</a> to assign the WinUSB driver to the USB device "WCH-Link" (Presumably on both Interfaces).

### Without VS Code
Follow the same steps and look up the exact commands used in `.vscode/tasks.json` instead.

### Linux Permissions
To make sure the flashing program (and the AsterTrack Application) has access to the controller usb device:
- For the firmware: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="16c0", ATTR{idProduct}=="05dc", MODE="0660", GROUP="sysplugdev"' | sudo tee -a /etc/udev/rules.d/40-astertrack.rules`
- For the bootloader: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="4348", ATTR{idProduct}=="55e0", MODE="0660", GROUP="sysplugdev"' | sudo tee -a /etc/udev/rules.d/40-astertrack.rules`
- For WCH-Link / WCH-LinkE: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="8010", MODE="0660", GROUP="sysplugdev"' | sudo tee -a /etc/udev/rules.d/40-astertrack.rules`
- `sudo udevadm control --reload-rules && sudo udevadm trigger`

For this to work, verify that you are already part of the sysplugdev group:
- `getent group | grep sysplugdev`
- or just `groups`

If not, make sure such a group exists and join it:
- `sudo groupadd --system sysplugdev`
- `sudo usermod -a -G sysplugdev $USER`
- then restart and verify you are now part of the group

## License
The AsterTrack Controller Firmware is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header. <br>
Notably, the usb_driver is licensed under MPL 2.0, as it is a full reimplementation of the USB stack for the CH32V307 with much more robust and fully-featured control transfers than the vendor-provided implementation. <br>
Additionally, any modifications to the dependencies are licensed under the license of the respective dependency as found in the licenses folder, with express permission to incorporate these modifications upstream.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.