@echo off

REM Build
python -m platformio run

REM Upload
python -m platformio run --target upload --upload-port COM3

REM Monitor
python -m platformio device monitor --port COM3