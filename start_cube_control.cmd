@echo off
setlocal
title LeArm Demonstration
where pythonw.exe >nul 2>&1
if errorlevel 1 (
    echo Python with Tkinter is required to open the LeArm control window.
    pause
    exit /b 1
)
python.exe -c "import tkinter, PIL, serial" >nul 2>&1
if errorlevel 1 (
    echo Install the PC app dependencies with:
    echo python -m pip install -r "%~dp0robot\requirements.txt"
    pause
    exit /b 1
)
start "" pythonw.exe "%~dp0robot\cube_control.pyw"
