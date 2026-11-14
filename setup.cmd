@echo off
setlocal
title LeArm Setup
set "VENV=%~dp0.venv"

if exist "%VENV%\Scripts\python.exe" goto install
where py.exe >nul 2>&1
if not errorlevel 1 (
    py -3 -m venv "%VENV%"
) else (
    python.exe -m venv "%VENV%"
)
if errorlevel 1 goto failed

:install
"%VENV%\Scripts\python.exe" -c "import tkinter"
if errorlevel 1 goto failed
"%VENV%\Scripts\python.exe" -m pip install -r "%~dp0robot\requirements.txt"
if errorlevel 1 goto failed
echo Setup complete. Run start_cube_control.cmd.
pause
exit /b 0

:failed
echo Setup failed. Check your Python installation, Tkinter and internet access.
pause
exit /b 1
