@echo off
echo Building and uploading via WiFi OTA...
python -m platformio run --environment ota --target upload
pause