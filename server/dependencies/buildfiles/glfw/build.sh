#!/usr/bin/env bash

# Make sure source directory is valid
SRC_PATH=source
if [[ ! -d "$SRC_PATH" ]]; then
	echo "$SRC_PATH does not exist!"
	exit 1
fi

# Create temporary directories
mkdir -p linux

echo -----------------------------------------
echo Building glfw
echo -----------------------------------------

pushd linux
cmake ../$SRC_PATH -DGLFW_BUILD_DOCS=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_EXAMPLES=OFF
make -j4 glfw
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

# Try to verify output
if [[ ! -f "linux/src/libglfw3.a" ]]; then
	echo "Build failed - linux/src/libglfw3.a does not exist!"
	exit 3
fi

# Copy resulting files to project directory
if [[ "${PWD:(-29)}" != "/dependencies/buildfiles/glfw" ]]; then
	echo "Not in dependencies/buildfiles/glfw subfolder, can't automatically install files into project folder!"
	exit 4
fi

echo "Installing dependency files..."

rm -r ../../include/GLFW
mkdir -p ../../include
cp -r source/include/GLFW ../../include/
rm  ../../lib/linux/libglfw3.a
mkdir -p ../../lib/linux
cp linux/src/libglfw3.a ../../lib/linux

echo "Now you can call clean.sh if everything succeeded."