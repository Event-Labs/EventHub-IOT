@echo off
set "IDF_PATH=E:\IEM ESP32\IEM\.espressif\v6.1\esp-idf"
set "IDF_TOOLS_PATH=E:\IEM ESP32\Espressif"
set "IDF_PYTHON_ENV_PATH=E:\IEM ESP32\Espressif\tools\python\v6.1\venv"
call "%IDF_PATH%\export.bat"
idf.py build
