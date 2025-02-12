@echo off & SETLOCAL ENABLEDELAYEDEXPANSION

set MODE=%1
call :tolower MODE

if not "%MODE%"=="build" (
	echo Cleaning source
	if exist source rd /s /q source
)

goto :EOF

:tolower
if NOT [!%1%!] == [] (
	for %%L IN (a b c d e f g h i j k l m n o p q r s t u v w x y z) DO SET %1=!%1:%%L=%%L!
)
goto :EOF