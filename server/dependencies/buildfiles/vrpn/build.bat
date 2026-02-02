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
echo Building VRPN
echo -----------------------------------------

if "%MODE%"=="debug" (
	set CRT=MultiThreadedDebug
) else (
	set CRT=MultiThreaded
)
:: -DCMAKE_MSVC_RUNTIME_LIBRARY is completely ignored unless you set this stupid policy, thank you CMake for nothing

pushd win\build\%MODE%
cmake -DCMAKE_MSVC_RUNTIME_LIBRARY=%CRT% -DCMAKE_POLICY_DEFAULT_CMP0091=NEW ^
	-DCMAKE_BUILD_TYPE=%MODE% -G "NMake Makefiles" ..\..\..\%SRC_PATH%
:: -DCMAKE_VERBOSE_MAKEFILE=ON
nmake vrpn quat
popd
pushd win\install\%MODE%
cmake -DCMAKE_INSTALL_COMPONENT=clientsdk -DCMAKE_INSTALL_PREFIX=. -P ..\..\build\%MODE%\cmake_install.cmake
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

:: Try to verify output
if not exist win\install\%MODE%\lib (
	echo Build failed - win\install\%MODE%\lib does not exist!
	exit /B
)

:: Copy resulting files to project directory
if "%CD:~-29%" neq "\dependencies\buildfiles\vrpn" (
	echo Not in dependencies\buildfiles\vrpn subfolder, can't automatically install files into project folder!
	exit /B
)

echo Installing dependency files...
if not exist ..\..\include\vrpn md ..\..\include\vrpn
robocopy win\install\%MODE%\include ..\..\include\vrpn /E /NFL /NDL /NJH /NJS
if not exist ..\..\lib\win\%MODE%\vrpn md ..\..\lib\win\%MODE%\vrpn
robocopy win\install\%MODE%\lib ..\..\lib\win\%MODE%\vrpn /E /NFL /NDL /NJH /NJS

echo Now you can call clean.bat if everything succeeded.