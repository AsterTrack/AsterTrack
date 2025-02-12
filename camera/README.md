# AsterTrack Camera

Firmware for the Raspberry Pi Zero in the AsterTrack Camera hardware. <br>

## Building

### Compilation
Cross-compilation is considered for the future - for now, compilation is only possible on the actual hardware itself. The general outline is as follows - see later sections for details on each individual step:
1. Build a `compile` image and flash it on an SD card with an SD card reader
2. Boot a Raspberry Pi (camera hardware or standalone) with that SD card and it will compile automatically
3. Wait until it finishes compilation and shuts down. This may take over 10min, and is signaled by the LED flashing slowly, then turning off and staying off.
4. Put the SD card back into an SD card reader
5. Use `data_read` followed by `storage_update` to extract the build artifacts from the SD card using and store them for later use
6. Build a regular image for tracking use - a `dev` image is recommended for now, but `wifi` and `normal` work, too. In addition to normal operation, a `dev`image is capable of re-compiling and iterative development should the need arise, without any downsides besides image size.
7. Flash that new image on the SD card and boot it with the Camera hardware. The first boot may take longer than followup boots.
8. The camera should eventually connect with the controller. If it does not, try pressing the flash button to soft-reset the controller, as there is currently an issue with establishing the connection.

### Building an Image

Enter the `setup` folder and execute the following commands:
1. `sudo ./setup_image.sh dev image_dev.img` to build a dev image (or choose a different image type)
2. Insert an SD card and find out the device path (e.g. /dev/sdaX, /dev/mmcblkX, etc.) of the SD card reader
3. `./flash_image.sh /dev/XXXXXX image_dev.img` to flash the SD card with the image. <br>
**THIS WILL ERASE ALL CONTENTS OF THE BLOCK DEVICE!** Make sure you specify the device path and check each and every time that it is the SD card as it may change. You can also directly use `dd` if you feel comfortable with it.

Note: Using these scripts, you'll never need to manually mount / open the SD card. Doing so might interfere with the scripts.

Note: Manual setup is no longer supported due to the benefits of automatic, repeatable image builds. While more limited than full Raspbian, piCore is about equivalent to Raspbian Lite with some differences and limitations in package management. Still, a full development environment is available, and so piCore and these setup scripts are fully embraced.

### Image Types

The Raspberry Pi runs a piCore operating system, which is configured using a set of scripts from a linux host PC. These scripts download an initial image from piCore, partition it properly, download and install all dependencies from piCore repositories, and configure the system and environment for the desired use. <br>
There are several image types that can be generated:
- `compile`: For a quick compile and offload operation without running TrackingCamera
- `dev-single`: Dev image without running TrackingCamera - just sets up wifi and SSH and waits
- `dev`: Dev image that can be used as a normal camera, but allows compilation and dev work via SSH when desired, and logs to SD card for later analysis
- `wifi`: Normal camera image that has wifi and SSH support enabled when setup via host software - e.g. to read logs, for a future server, etc.
- `normal`: Normal camera image, as small and quick as can be, no wifi support

Feel free to check `setup_image.sh` to check what each image type does exactly.

### Image Tools

There are more tools at your disposal:
- `./data_read.sh /dev/XXXXXX` will read the user data from the SD card into the `data` folder, which includes the configuration and environment as set up by the scripts as well as the home directory with build artifacts and logs.
- `./data_write.sh /dev/XXXXXX` will update the user data on the SD card from the `data` folder, which allows you to manually edit it easily without dealing with piCores packed user data file.
- `./storage_update.sh` will read the build artifacts and wifi credentials from the `data` folder (read using `data_read`) and store it in `storage`. These will be automatically setup in future image builds.
- `./storage_apply.sh` will write the build artifacts and wifi credentials stored in `storage` into the `data` folder for you to write to the SD with `data_write`.

The `storage/wifi.db` is root-protected by default and stores wifi credentials for automatic connection (currently only on `dev-single` images).
Each line stores one known network: `<SSID> <Password> WPA` where SSID is space-escaped (e.g. "My\ Home\ Network"). <br>
Note: This is not required if you pass the wifi credentials on-demand using the AsterTrack Application Interface. <br>

## License
The AsterTrack Camera firmware is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.