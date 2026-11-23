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
echo REAL ROBOT PICK - camera and arm must use DIFFERENT ports.
set "CAMERA_PORT=COM5"
set /p CAMERA_PORT=Camera COM port [COM5]:
set /p ROBOT_PORT=Robot arm COM port:
if "%ROBOT_PORT%"=="" goto badport
if /i "%CAMERA_PORT%"=="%ROBOT_PORT%" goto badport
"%YOLO_PY%" "%~dp0robot\yolo_pick.py" --camera-port "%CAMERA_PORT%" --robot-port "%ROBOT_PORT%" --execute --sort
pause
exit /b
:badport
echo Enter a different robot arm COM port. No program started.
pause
exit /b 1
