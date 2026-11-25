@echo off
REM Launch FDC Studio on Windows. First run creates a virtual environment next to this file and installs the requirements.
REM   Usage: run_fdc_studio.bat [--simulate]
set HERE=%~dp0
set VENV=%HERE%.venv
if not exist "%VENV%\Scripts\python.exe" python -m venv "%VENV%"
"%VENV%\Scripts\python.exe" -c "import PyQt5, pyqtgraph, serial, numpy" 2>nul || "%VENV%\Scripts\pip.exe" install -r "%HERE%requirements.txt"
"%VENV%\Scripts\python.exe" "%HERE%fdc_studio.py" %*
