FROM debian:bookworm AS build

COPY setup_toolchain_llvm.sh /
RUN /setup_toolchain_llvm.sh

# Create fresh image to copy LLVM toolchain to
FROM debian:bookworm
COPY --from=build /opt/toolchain /opt/toolchain

RUN apt-get update -y
RUN apt-get install -y make cmake autoconf automake libtool unzip wget
RUN apt-get install -y libgl1-mesa-dev libglu1-mesa-dev libglew-dev libwayland-dev libxkbcommon-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev libudev-dev libdbus-1-dev libturbojpeg0-dev

ENV PATH=/opt/toolchain/bin:$PATH
ENV CC=/opt/toolchain/bin/clang
ENV CXX=/opt/toolchain/bin/clang++
