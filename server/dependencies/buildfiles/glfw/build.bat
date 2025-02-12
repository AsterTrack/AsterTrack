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

if not exist win\%MODE% md win\%MODE%

echo -----------------------------------------
echo Building glfw
echo -----------------------------------------

:: -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded Doesn't work, completely ignored, thank you CMake
:: So had to read previous flags from CMD and modfify MDx to MTx...

pushd win\%MODE%
cmake -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
	-DCMAKE_C_FLAGS_DEBUG="/MTd /Zi /Ob0 /Od /RTC1" -DCMAKE_C_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG" ^
	-DCMAKE_CXX_FLAGS_DEBUG="/MTd /Zi /Ob0 /Od /RTC1" -DCMAKE_CXX_FLAGS_RELEASE="/MT /O2 /Ob2 /DNDEBUG" ^
	-DGLFW_BUILD_DOCS=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_EXAMPLES=OFF ^
	-DCMAKE_BUILD_TYPE=%MODE% -G "NMake Makefiles" ..\..\%SRC_PATH%
nmake glfw VERBOSE=1
popd

echo -----------------------------------------
echo Build completed
echo -----------------------------------------

:: Try to verify output
if not exist win\%MODE%\src\glfw3.lib (
	echo Build failed - win\%MODE%\src\glfw3.lib does not exist!
	exit /B
)

:: Copy resulting files to project directory
if "%CD:~-29%" neq "\dependencies\buildfiles\glfw" (
	echo Not in dependencies\buildfiles\glfw subfolder, can't automatically install files into project folder!
	exit /B
)

echo Installing dependency files...
if not exist ..\..\include md ..\..\include
robocopy source\include ..\..\include /E /NFL /NDL /NJH /NJS
if not exist ..\..\lib\win md ..\..\lib\win
if not exist ..\..\lib\win\%MODE% md ..\..\lib\win\%MODE%
copy win\%MODE%\src\glfw3.lib ..\..\lib\win\%MODE%

echo Now you can call clean.bat if everything succeeded.

goto :EOF

:tolower
for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
goto :EOF

:getabsolute
set %1=%~f2
goto :eof