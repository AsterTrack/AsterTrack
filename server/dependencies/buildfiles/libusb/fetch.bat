@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

:: Frozen v1.0.30-prerelease
::set FETCH_URL=https://github.com/libusb/libusb/archive/refs/tags/v1.0.29.zip
set FETCH_URL=https://github.com/libusb/libusb/archive/102ab657bbc20b97579c477149739a6821a45e7a.zip
set FETCH_VERSION=v1.0.30-prerelease
set FETCH_NAME=libusb
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