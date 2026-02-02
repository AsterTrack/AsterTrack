@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Make sure we have the environment variables necessary to build
if NOT "%VSCMD_ARG_TGT_ARCH%" == "x64" (
	echo Please use the x64 Native Tools Command Prompt for vcvars64 environment!
	exit /B
)

:: Make sure source directory is valid
set SRC_PATH=source
if not exist "%SRC_PATH%" (
	echo %SRC_PATH% does not exist!
	exit /B
)

:: Select release or debug mode
set MODE=%1
if "%MODE%"=="" (
	set MODE=release
) else (
	if NOT "%MODE%"=="release" (
		if NOT "%MODE%" == "debug" (
			echo Invalid mode %MODE%, expecting 'release' or 'debug'
			exit /B
		)
	)
)

if not exist win\build\%MODE% md win\build\%MODE%
if not exist win\install\%MODE% md win\install\%MODE%

echo -----------------------------------------
echo Building libusb
echo -----------------------------------------

:: libusb provides configurations to select MT instead of MD
if "%MODE%"=="debug" (
	:: Could use Debug-Hotplug-MT
	set CONFIG=Debug-MT
) else (
	:: Could use Release-Hotplug-MT
	set CONFIG=Release-MT
)

MSBuild.exe %SRC_PATH%\msvc\libusb_dll.vcxproj -property:RuntimeLibrary=%CRT% -property:EnableASAN=false ^
	-property:Configuration=%CONFIG% -property:IntDir=..\..\win\build\%MODE%\ -property:OutDir=..\..\win\install\%MODE%\

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

:: Try to verify output
if not exist win\install\%MODE%\libusb-1.0.dll  (
	echo Build failed - libusb-1.0.dll does not exist!
	exit /B
)


:: Copy resulting files to project directory
if "%CD:~-31%" neq "\dependencies\buildfiles\libusb" (
	echo Not in dependencies\buildfiles\libusb subfolder, can't automatically install files into project folder!
	exit /B
)

echo Installing dependency files...
if not exist ..\..\include\libusb md ..\..\include\libusb
copy  %SRC_PATH%\libusb\libusb.h ..\..\include\libusb
if not exist ..\..\lib\win\%MODE%\libusb md ..\..\lib\win\%MODE%\libusb
robocopy win\install\%MODE% ..\..\lib\win\%MODE%\libusb /lev:0 /NFL /NDL /NJH /NJS

echo Now you can call clean.bat if everything succeeded.