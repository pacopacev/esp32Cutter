@echo off
echo Building and uploading via USB...
python -m platformio run --environment usb --target upload
pause