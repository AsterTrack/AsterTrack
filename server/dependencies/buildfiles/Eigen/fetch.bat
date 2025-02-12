@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

::set FETCH_URL=https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
:: Need newer version than old 3.4.0, change to fixed version as soon as one is released
set FETCH_URL=https://gitlab.com/libeigen/eigen/-/archive/master/eigen-master.zip
set FETCH_VERSION=master
set FETCH_NAME=eigen
set FETCH_ARCHIVE=source.zip

if exist "source\srcversion" (
	set /p SRC_VERSION=<"source\srcversion"
	if not "!SRC_VERSION!"=="%FETCH_VERSION%" (
		del "source\srcversion"
	) else (
		echo Already downloaded %FETCH_NAME% %FETCH_VERSION%!
	)
)

if not exist "source\srcversion" (
	echo Downloading %FETCH_NAME% %FETCH_VERSION% source
	curl.exe -L -o %FETCH_ARCHIVE% %FETCH_URL%
	if not exist %FETCH_ARCHIVE% (
		echo Failed to download source!
		exit /B
	)

	if exist "source" rd /s /q "source"

	echo Unpacking %FETCH_NAME% %FETCH_VERSION% source
	tar -xf %FETCH_ARCHIVE%
	move "%FETCH_NAME%-*" "source"

	del %FETCH_ARCHIVE%
	(echo %FETCH_VERSION%) > source/srcversion
	echo Done downloading %FETCH_NAME% %FETCH_VERSION%!
)

:: Copy files to project directory
if "%CD:~-30%" neq "\dependencies\buildfiles\Eigen" (
	echo Not in dependencies\buildfiles\Eigen subfolder, can't automatically install files into project folder!
	exit /B
)

echo Installing dependency files...
if not exist ..\..\include\Eigen md ..\..\include\Eigen
robocopy source\Eigen ..\..\include\Eigen\ /E /NFL /NDL /NJH /NJS /MIR
if not exist ..\..\include\unsupported\Eigen md ..\..\include\unsupported\Eigen
robocopy source\unsupported\Eigen ..\..\include\unsupported\Eigen\ /E /NFL /NDL /NJH /NJS /MIR
