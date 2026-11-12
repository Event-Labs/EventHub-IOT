    @echo off
title EventHub QR Check-in Station (ESP32-S3 OV5640 USB)
cd /d "%~dp0"

echo ========================================================
echo   EventHub QR Check-in Station - USB Streamer
echo   Hardware: ESP32-S3 + OV5640 Cam on COM6 @ 2Mbaud
echo ========================================================
echo.

set PYTHON="E:\IEM ESP32\Espressif\tools\python\v6.1\venv\Scripts\python.exe"

if not exist %PYTHON% (
    echo [ERROR] Python environment not found at: %PYTHON%
    pause
    exit /b 1
)

echo Releasing COM6 port from any previous lingering sessions...
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.Name -like 'python*' -and $_.CommandLine -like '*stream_app.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" >nul 2>&1
ping -n 2 127.0.0.1 >nul

echo Starting Local USB Stream Server & Web UI...
start "" "http://localhost:5000"
%PYTHON% -u stream_app.py --port COM6 --baud 2000000 --http-port 5000

pause
