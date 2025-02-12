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
	set MODE
	call :tolower MODE
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

:: Sadly MSBuild doesn't support overriding the CRT used in a project file.
if "%MODE%"=="debug" (
	set CRT=MultiThreadedDebug
) else (
	set CRT=MultiThreaded
)
:: So this does the actual work of enforcing MT instead of MD
:: Note this will have two CRTs, one in the libusb DLL, one in the main program
:: However the libusb interface is OK with two separate CRTs
SET _CL_=/MT

MSBuild.exe %SRC_PATH%\msvc\libusb_dll.vcxproj -property:Configuration=%MODE% -property:IntDir=..\..\win\build\%MODE%\ -property:OutDir=..\..\win\install\%MODE%\ -property:RuntimeLibrary=%CRT%

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

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF

:getabsolute
set %1=%~f2
goto :eof