@echo off

:: Make sure we have the environment variables necessary to build
if NOT "%VSCMD_ARG_TGT_ARCH%" == "x64" (
	echo Please use the x64 Native Tools Command Prompt for vcvars64 environment!
	exit /B
)

set MODE=release
:: release, debug
if NOT "%1" == "" (
	set MODE=%1
)

set DEPENDENCIES=Eigen,glfw,libusb,vrpn
(for %%d in (%DEPENDENCIES%) do (
	pushd buildfiles\%%d
	echo =====================================================================
	echo Downloading, building and installing %%d...
	fetch.bat
	if exist build.bat (
		build.bat %MODE%
	)
	popd
))