@echo off

set DEPENDENCIES=Eigen,glfw,libusb,vrpn
(for %%d in (%DEPENDENCIES%) do (
	pushd buildfiles\%%d
	echo =====================================================================
	echo Downloading %%d...
	fetch.bat
	popd
))