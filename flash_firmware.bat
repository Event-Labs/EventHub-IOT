@echo off
title Flash EventHub QR Check-in Firmware
cd /d "%~dp0"

echo ========================================================
echo   Flash EventHub Firmware to ESP32-S3 (COM6)
echo ========================================================
echo.

echo Releasing COM6 from any stream or serial processes...
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.Name -like 'python*' -and $_.CommandLine -like '*stream_app.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" >nul 2>&1
ping -n 2 127.0.0.1 >nul

set "IDF_PATH=E:\IEM ESP32\IEM\.espressif\v6.1\esp-idf"
set "IDF_TOOLS_PATH=E:\IEM ESP32\Espressif"
set "IDF_PYTHON_ENV_PATH=E:\IEM ESP32\Espressif\tools\python\v6.1\venv"

call "%IDF_PATH%\export.bat"
idf.py -p COM6 flash
if errorlevel 1 (
    echo.
    echo ========================================================
    echo [ERROR] Flash THAT BAI! Kiem tra xem COM6 co dang bi chiem boi phan mem khac khong.
    echo ========================================================
    if not "%NO_PAUSE%"=="1" pause
    exit /b 1
)

echo.
echo ========================================================
echo Flash hoan tat! Nhan phim bat ky de thoat.
echo ========================================================
if not "%NO_PAUSE%"=="1" pause
