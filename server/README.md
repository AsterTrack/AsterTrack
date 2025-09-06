# AsterTrack Application

This is the main AsterTrack Application running on the host PC (Linux or Windows 10+). It includes both the server that runs in the background, communicates with the tracking system hardware and performs the tracking, as well as the user interface component which is loaded on-demand. <br>
While they are tightly integrated, the UI is also clearly separated from the server in that the server can run without the UI code being loaded at all.

## Building

### Requirements

#### Linux
If you want to build & develop normally:
- Depending on the compiling toolchain (COMPILER input to Makefile):
  - clang-std (recommended): `clang`
  - clang-libc: `clang`, `libc++`, `libomp5`/`openmp`
  - gcc: `gcc`, `g++`
- If using clangd for development: `clangd`, `bear`)
- Build tools: `cmake`, `autoconf`, `automake`, `libtool`, `unzip`, `wget`
- OpenGL **development** libraries: `libgl1-mesa`, `libglu1-mesa`, `libglew`
- Wayland **development** libraries: `libwayland`, `libxkbcommon`
- X11 **development** libraries: `libxcursor`, `libxrandr`, `libxinerama`, `libxi`
- System **development** libraries: `libudev`, `libdbus`
- TurboJPEG: `libturbojpeg`/`libturbojpeg0`
- wget for fetching dependencies

If you want to build a clang-std + release version with limited development capabilities:
- Use `Build inside Debian Environment` task
  - just depends on `git`
  - needs superuser privileges
  - will setup a debian 12 environment (GLIBC v2.36) using debootstrap
  - will chroot into it and install all build dependencies
  - will compile its own clang 21.1 toolchain
  - will automatically compile server in release mode (others not supported)

This is intended for building a binary distribution compatible with most modern Linux distributions - NOT for development!


#### Windows
**1. MSVC compiler** <br>
If you have Visual Studio installed, you're good <br>
Otherwise, to download the minimal setup (only the build tools):
  - Download the <a href="https://visualstudio.microsoft.com/visual-cpp-build-tools/">Visual Studio Build Tools</a>
  - On installation, select "Desktop development with C++"
  - Then select the following components on the right (can unselect the others):
    - `MSVC`, `Windows 11 SDK`, `C++ CMake`

Also make sure that <a href="https://docs.microsoft.com/en-us/cpp/build/building-on-the-command-line#developer_command_file_locations">vcvars64.bat</a> is executable:
  - Add it to the path, depending on your version, e.g: <br>
    `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build`
  - Alternatively, modify .vscode/tasks.json to point to the vcvars64.bat file

**2. turbojpeg** <br>
Get the latest 3.x release from the <a href="https://github.com/libjpeg-turbo/libjpeg-turbo/releases">libjpeg-turbo github</a> (`libjpeg-turbo-3.x.x-vc64.exe`) <br>
You can run the file to install turbojpeg system-wide, just make sure to update the paths in CMakeLists.txt (it defaults to `C:/libjpeg-turbo64`). <br>
Alternatively, you can extract it with 7-Zip and put the library files (`turbojpeg.lib` and `turbojpeg-static.lib`) into `dependencies/lib/win` as well as the include file `turbojpeg.h` into `dependencies/include`.

#### VS Code
Additional extensions for development in VS Code: <br>
<i>Recommended</i>: clangd (llvm-vs-code-extensions) and CodeLLDB (vadimcn) extensions. Build Clangd Database using tasks. <br>
Alternatively: C/C++ (microsoft)

### Dependencies
To create a minimal build of the dependencies custom build scripts are provided. <br>
Note none of these are installing any systemwide files, after the clean operation the only changed files are in the projects include/ and lib/ folders. <br>

#### VS Code
Run Task "Build All Dependencies" and select "normal" or "release" as mode.
If successful (try building the program itself), you can run "Clean All Dependencies".

#### Manually
<b>For Windows</b> Make sure you are in vcvars64 environment of MSVC. Usually that means the "x64 Native Tools Command Prompt" that is setup after install of the Build Tools. Make sure NOT to use the x86 version.

While in the dependencies subfolder, you can fetch and build all dependencies as follows (*.bat on Windows and *.sh on Linux):
  - `build.[sh/bat] [release/debug]`
  - If successful: `clean.[sh/bat] all`

To make it easier to debug problems, you can enter each individual dependencies/buildfiles/* directory and execute the corresponding scripts:
  - `fetch.[sh/bat]`
  - `build.[sh/bat] source [release/debug]`
  - If successful: `clean.[sh/bat] source all`


NOTE: On windows, both release and debug builds of the application by default expect the corresponding release and debug builds of the dependencies. This can be disabled if you do not need to debug the dependencies:  <br>
In CMakeLists.txt, change `set(LIB_DIR ${PLT_LIB_DIR}/debug)` to `set(LIB_DIR ${PLT_LIB_DIR}/release)` <br>

### Compilation

#### VS Code
Run task "Build" or "Build (clangd)" if you use the clangd extension. Then select appropriate build configuration (recommended: clang, release-verify).

#### Manually
**Linux** (in server directory):
  - `make BUILD_TYPE=release-verify -j$(nproc --ignore=2)`

**Windows** (in server directory, in vcvars64 environment):
  - `mkdir build & cd build`
  - `cmake -DCMAKE_BUILD_TYPE=Release ..`
  - `msbuild astertrack.sln -p:Configuration=Release`

### Linux Permissions
To make sure the program has access to the controller usb device (both for tracking and for flashing):
- For the firmware: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="16c0", ATTR{idProduct}=="05dc", MODE="0660", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/00-usb-permissions.rules`
- For the bootloader: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="4348", ATTR{idProduct}=="55e0", MODE="0660", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/00-usb-permissions.rules`
- `sudo udevadm control --reload-rules`

For this to work, verify that you are already part of the plugdev group:
`getent group | grep plugdev`

If not, make sure such a group exists and join it:
- `sudo groupadd plugdev`
- `sudo usermod -a -G plugdev $USER`
- and then restart (or log out and back in)

Alternatively, execute the program with sudo (not recommended)

## License
The AsterTrack Application is licensed under the LGPL v3, but parts of the codebase are licensed under the MIT license or the MPL 2.0. This can be determined on a file-by-file basis by looking at the file header. <br>
Notably, most of the multi-camera calibration system is licensed under the MPL 2.0. <br>
Additionally, any modifications to the dependencies are licensed under the license of the respective dependency as found in the licenses folder, with express permission to incorporate these modifications upstream.

The following merely describes the intent behind this seggregation: <br>
**MIT:**
In general, smaller, self-contained components and utilities that might be useful to other projects are licensed under the MIT license. <br>
**MPL 2.0:**
Some bigger algorithmic components are licensed under the MPL 2.0 with the intent that they can be repurposed with minor efforts for other projects. These files are not explicitly designed to be repurposed and some references to LGPL-licensed code might have to be removed. However, the author (Seneral) deemed those components potentially valuable to further open source development in other fields. <br>
**LGPL v3**:
The majority of the source code that is specific to AsterTrack or an optical tracking system just like it is licensed under the LGPL v3, with file-by-file exceptions as previously noted. <br>

If you wish to use a source code file in part or in full that currently is licensed too restrictively for you, feel free to contact me and I will consider changing the license of at least the source code over which I still hold copyright.

### Supplementary files
All non-source-code-files in `store` and `resources` are licensed under the MIT License.
