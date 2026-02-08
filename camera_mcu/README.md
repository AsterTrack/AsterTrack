# AsterTrack Camera MCU

Firmware for the STM32G030 in the AsterTrack Camera hardware. <br>
It is responsible for timing and frame sync, most of the UART comms, all user interactivity (buttons and LEDs), and storing information about the hardware.

## Building

The build system is designed for and tested primarily on Linux, but you can compile it on Windows as well.

### Requirements

#### Windows: WSL / MSYS2 / Cygwin
While the toolchain itself is available for Windows, the build system uses GNU Make, and thus you need to install either MSYS2 or Cygwin, or you need to use Windows Subsystem for Linux (WSL) and follow the Linux commands. <br>
While MSYS2 and Cygwin both work fine to compile, some other development and debugging tools have better support on Linux, so WSL is better suited for that. <br>
If you choose to use MSYS2 or Cygwin (both of them are equally good options), you only need to install `make` in either of them, and update the paths in `.vscode/tasks.json` to their respective `bash.exe`. <br>
You can install `make` in MSYS2 by opening the MSYS2 terminal and running `pacman -S make`. <br>
You can install `make` in Cygwin by selecting the `make` packet in the provided setup installer (you may have to set the filter to 'ALL' and then search for 'make').

**GNU ARM Embedded Toolchain**: <br>
Download the `AArch32 bare-metal target (arm-none-eabi)` toolchain for your platform from <a href="https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads">the ARM website</a>. Versions v14 and v15 have been confirmed to work. <br>
The toolchain can also be built manually using <a href="https://github.com/riscv-collab/riscv-gnu-toolchain">this source repository</a> or using <a href="https://crosstool-ng.github.io/">crosstools-ng</a>. <br>
You may place the toolchain in e.g. `/opt` or `C:/` and update the TOOL_PATH in `toolpath.make`.

### Compilation
Run task `Build` to build the firmware for the STM32G030. <br>
You may run `Build (clangd)` on Linux instead if you have `bear` installed and use the clangd LSP. This will update the `compile_commands.json` file required to give clangd an accurate understanding of the project. You may need to delete the build folder first to force a full-rebuild if you used the normal build task before.

## Flashing

### Automatic Flashing using AsterTrack Server Software (recommended)

This method uses the built-in utility of the Server Software to instruct the Raspberry Pi in the Camera to flash the MCU via its bootloader. No extra hardware is required. <br>
1. Connect to the controller the camera is connected to. You should see it in the Devices View.
2. Select the camera for firmware update using the button - it should appear under "Camera Firmware Update". <br>
3. Select the camera-mcu firmware file under "Camera Firmware Update".
4. Hit the "Flash" button to start the flashing process - ensure power is not accidentally cut after this!
5. After the transfer to the camera, the cameras LEDs should turn pink to signal the flashing process is ongoing.

### About debug probes
You can attach an ST-Link-compatible probe to the debug pins on the camera PCB. These are labelled `+`: 3.3V, `d`: SW Data IO, `c`: SW Clock, `-`: Ground.
Do NOT connect the 3V pin if you connect the camera to a controller or otherwise provide power to it - only use it if there is nothing else connected and you want to power the chip from the programmer. <br>

#### Boot Modes
The STM32G030 has two boot modes that can be configured, since SWCLK and BOOT0 share a pin. <br>
The SBC may use the BOOT0 pin to force the MCU into bootloader mode for flashing, but only does so when the MCU is unresponsive and assumed to be bricked - otherwise, the SBC requests the MCU over I2C to switch to the bootloader itself. <br>
The SWCLK is used whenever you have a probe attached to the debug pins. <br>
By default, the STM32G030 disables BOOT0 in favour of SWCLK. AsterTrack decided to prefer the ability of the SBC to flash the MCU in case it ever gets bricked.
So that is the boot mode that should be configured. If that is not the case, the camera will flash blue for 500ms on startup. <br>
To switch between both modes, you can hold down one of the two face buttons while the camera is powered off, and then provide power to the camera.
The button you hold will charge up in either blue or green for 1 second, then hold its color for another 2s, giving you the chance to release the button to accept the change in boot mode.
If you do release the button in that 2s time window, the boot mode will get programmed to the option bytes, and the button will flash in that color a few times (backround color red), before the MCU starts up as usual - unless that boot mode was already set, then it just continues immediately after you release the button.
If you do not release the button, at 3s after startup, the buttons will flash red (one slow, one quick) to let you know the boot change has been aborted and you should release the button to let the boot process continue. <br>
The boot mode enabling the BOOT0 pin (the intended mode for AsterTrack) is represented by the green color in this scenario. The alternative boot mode disabling BOOT0 in favor of SWD (the default for the chip) is represented by blue, corresponding to the flash of blue you see if that boot mode is configured. <br>
In practice, there has been no issues observed using a probe for both debugging and flashing so far, even in the boot mode that enables BOOT0, as that pin becomes SWCLK anyway right after reset. However, in case you ever experience problems debugging, you have the tools to switch to the alternative boot mode available to you.

#### Flashing using st-flash / STM32_Programmer_CLI / OpenOCD
Simply use the tasks `Flash (st-flash)`, `Flash (STM32_Programmer_CLI)` or `Flash (OpenOCD)` to flash the program when you have an ST-Link-compatible probe attached. <br>
All programs are available for both Linux and Windows, but <a href="https://github.com/stlink-org/stlink/releases/">stlink/st-flash</a> is open source and can just be downloaded, whereas <a href="https://www.st.com/en/development-tools/stm32cubeprog.html">STM32_Programmer_CLI</a> requires you to give up your email to ST to download. <br>
OpenOCD is required only when you intend to debug as well.

#### Debugging using OpenOCD
Install the `marus25.cortex-debug` extension and select the debug configuration `STM32G030 SWD (STLink)` with an ST-Link-compatible probe attached. <br>
You may need to update `armToolchainPath` in `.vscode/launch.json` to point to your ARM toolchain, though it is not required. <br>
It is recommended to install OpenOCD using your distributions package manager, otherwise, you may need to update `serverpath` in `.vscode/launch.json` as well. <br>
**Windows:** The easiest way to install OpenOCD is to download the <a href="https://github.com/xpack-dev-tools/openocd-xpack/releases">xPack release</a>.
Then, set `serverpath` in `.vscode/launch.json` to the extracted folder. Similarly, update the command of the `Flash (OpenOCD)` task in `.vscode/tasks.json` to point to the openocd binary.
Finally, you need to use <a href="https://zadig.akeo.ie/">Zadig</a> to assign the WinUSB driver to the USB device "STM32 STLink". Then, you should be able to both flash and debug with OpenOCD.

### Without VS Code
Follow the same steps and look up the exact commands used in `.vscode/tasks.json` instead.

### Linux Permissions
You may need to make sure your debug probe is accessible by your user. If you install OpenOCD via your distributions package manager, check if it hasn't already installed udev rules for you - you may just need to restart. Otherwise, find out the exact VID and PID of your debug probe.
- E.g. for ST-Link V2: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0660", GROUP="sysplugdev"' | sudo tee -a /etc/udev/rules.d/50-usb-dev.rules`
- `sudo udevadm control --reload-rules && sudo udevadm trigger`

For this to work, verify that you are already part of the sysplugdev group:
- `getent group | grep sysplugdev`
- or just `groups`

If not, make sure such a group exists and join it:
- `sudo groupadd --system sysplugdev`
- `sudo usermod -a -G sysplugdev $USER`
- then restart and verify you are now part of the group

## License
The AsterTrack Camera MCU Firmware is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header. <br>
Additionally, any modifications to the dependencies are licensed under the license of the respective dependency as found in the licenses folder, with express permission to incorporate these modifications upstream.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.