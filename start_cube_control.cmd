@echo off
setlocal
title LeArm Demonstration
set "PYTHON=%~dp0.venv\Scripts\python.exe"
set "PYTHONW=%~dp0.venv\Scripts\pythonw.exe"

if not exist "%PYTHON%" goto setup
if not exist "%PYTHONW%" goto setup
"%PYTHON%" -c "import tkinter, PIL, serial" >nul 2>&1
if errorlevel 1 goto setup
start "" "%PYTHONW%" "%~dp0robot\cube_control.pyw"
exit /b %errorlevel%

:setup
echo Run setup.cmd first to install the LeArm app dependencies.
pause
exit /b 1
