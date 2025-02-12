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
echo Building VRPN
echo -----------------------------------------

if "%MODE%"=="debug" (
	set CRT=MultiThreadedDebug
) else (
	set CRT=MultiThreaded
)
:: -DCMAKE_MSVC_RUNTIME_LIBRARY=%CRT% Doesn't work, completely ignored, thank you CMake
:: So had to read previous flags from CMD and modfify MDx to MTx...

pushd win\build\%MODE%
cmake ^
	-DCMAKE_C_FLAGS_DEBUG="/MTd /Zi /Ob0 /Od /RTC1" -DCMAKE_C_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG" ^
	-DCMAKE_CXX_FLAGS_DEBUG="/MTd /Zi /Ob0 /Od /RTC1" -DCMAKE_CXX_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG" ^
	-DCMAKE_BUILD_TYPE=%MODE% -G "NMake Makefiles" ..\..\..\%SRC_PATH%
nmake vrpn quat VERBOSE=1
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

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF

:getabsolute
set %1=%~f2
goto :eof