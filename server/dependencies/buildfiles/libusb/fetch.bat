@echo off & SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

::set FETCH_URL=https://github.com/Seneral/libusb/archive/master.zip
:: Frozen v1.0.26
set FETCH_URL=https://github.com/libusb/libusb/archive/aa633daaac2986f5361ba86ad7e0a3315156455e.zip
:: Issue: Streaming (switching configuration) causes stalling on interrupt and control endpoints
:: Later version did not fix the issue (windows only issue)
::set FETCH_URL=https://github.com/libusb/libusb/archive/6bf2db6feaf3b611c9adedb6c4962a07f5cb07ae.zip
set FETCH_VERSION=v1.0.26+msvc-revamp
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