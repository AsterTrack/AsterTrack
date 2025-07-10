@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Frozen v07.36
set FETCH_URL=https://github.com/vrpn/vrpn/archive/refs/tags/v07.36.zip
:: Custom modification to expose more connectivity information
set FETCH_URL=https://github.com/Seneral/vrpn/archive/3474fff0c3bb12b56af16a5f7ed469558e54a597.zip
set FETCH_VERSION=v07.37-custom
set FETCH_NAME=vrpn
set FETCH_ARCHIVE=source.zip

if exist "source\srcversion" (
	set /p SRC_VERSION=<"source\srcversion"
	if not "!SRC_VERSION!"=="%FETCH_VERSION%" (
		del "source\srcversion"
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