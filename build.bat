@echo off
color 0A
title ESP32 USB Build

echo Building firmware via USB (COM3)...
python -m platformio run --target upload --upload-port COM3
if errorlevel 1 (
    echo Build/Upload failed!
    pause
    exit /b 1
)

echo.
echo Firmware uploaded successfully via USB.
echo Opening serial monitor...
timeout /t 2 /nobreak > nul
python -m platformio device monitor --port COM3