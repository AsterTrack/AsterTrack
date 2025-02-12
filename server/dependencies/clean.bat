@echo off

set MODE=build
:: all, build
if NOT "%1" == "" (
	set MODE=%1 
)

set DEPENDENCIES=Eigen,glfw,libusb,vrpn
(for %%d in (%DEPENDENCIES%) do (
	pushd buildfiles\%%d
	if exist clean.bat (
		clean.bat %MODE%
	)
	popd
))