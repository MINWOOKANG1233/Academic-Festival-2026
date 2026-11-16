@echo off
setlocal
cd /d "%~dp0"
set "YOLO_PY=%~dp0.venv\Scripts\python.exe"
if not exist "%YOLO_PY%" set "YOLO_PY=%~dp0..\.venv-yolo\Scripts\python.exe"
if not exist "%YOLO_PY%" (
    echo Run setup.cmd and setup_yolo.cmd first.
    pause
    exit /b 1
)
set /p CAMERA_PORT=Camera COM port:
"%YOLO_PY%" "%~dp0robot\yolo_pick.py" --camera-port "%CAMERA_PORT%"
pause
