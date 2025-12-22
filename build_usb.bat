@echo off
echo Building and uploading via USB...
python -m platformio run --environment usb --target upload

echo Opening serial monitor...
timeout /t 2 /nobreak > nul
python -m platformio device monitor --port COM3
pause