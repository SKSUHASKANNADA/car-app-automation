@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" car_control_app.py
) else (
    python car_control_app.py
)
pause
