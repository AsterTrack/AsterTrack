#!/bin/bash

TOOLCHAIN=/opt/toolchain

# Dependencies for compiling LLVM
apt-get update -y
apt-get install -y gcc g++ ninja-build git cmake python3 python3-yaml binutils-dev

# Fetch LLVM source
git config --global --add remote.origin.fetch '^refs/heads/users/*'
git config --global --add remote.origin.fetch '^refs/heads/revert-*'
git clone --depth 1 --branch llvmorg-22.1.8 https://github.com/llvm/llvm-project.git

pushd /llvm-project

# Configure for all required + a few extra like clangd, but only for x86
cmake -S llvm -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=$TOOLCHAIN \
	-DCLANG_CONFIG_FILE_USER_DIR=$TOOLCHAIN/config \
	-DLLVM_ENABLE_PROJECTS='clang;lld;clang-tools-extra' \
	-DLLVM_ENABLE_RUNTIMES='libunwind;libcxxabi;libcxx;compiler-rt;openmp' \
	-DLIBCXX_STATICALLY_LINK_ABI_IN_STATIC_LIBRARY=ON \
	-DLIBCXXABI_STATICALLY_LINK_UNWINDER_IN_STATIC_LIBRARY=ON \
	-DLIBOMP_ENABLE_SHARED=OFF \
	-DLIBCXX_ENABLE_STATIC=ON \
	-DLIBCXX_ENABLE_STATIC_ABI_LIBRARY=ON \
	-DLIBUNWIND_ENABLE_STATIC=ON \
	-DLIBCXX_USE_COMPILER_RT=ON \
	-DLIBCXXABI_USE_COMPILER_RT=ON \
	-DLIBCXXABI_USE_LLVM_UNWINDER=ON \
	-DCOMPILER_RT_USE_LLVM_UNWINDER=ON \
	-DCLANG_DEFAULT_RTLIB=compiler-rt \
	-DCLANG_DEFAULT_UNWINDLIB=libunwind \
	-DCLANG_DEFAULT_CXX_STDLIB=libc++ \
	-DLLVM_TARGETS_TO_BUILD='X86' \
	-DLIBOMP_LDFLAGS='-Wl,--no-as-needed' # Patch, see below

# libclang_rt.builtins.a contains divxc3, which depends on libm
# clang appends that after OpenMPs -lm, while --as-needed is active.
# causing ld to discard libm before libclang_rt.builtins.a is considered
# See also:
# https://github.com/llvm/llvm-project/issues/43749
# https://github.com/llvm/llvm-project/issues/29026
# https://reviews.llvm.org/D49514

# Libc is not used (we use glibc), and it won't compile on Debian 12 or lower anyway
# Its tests are referencing chrono but are missing the include to /llvm-project/libcxx/include

# We rely on the default parameters of the custom LLVM build
#  to ensure libc++ / compiler-rt / libunwind are statically linked
# Specifying them via e.g. environment variables doesn't work well,
#  namely libusbs libtool strips out --rtlib=compiler-rt

# Compile and install
ninja -C build runtimes
ninja -C build install-runtimes
ninja -C build install

popd