@echo off
REM Modified version with hardcoded password
REM Usage: extract_archive_no_password.bat <gma_file>

if "%~1"=="" (
    echo Error: GMA file not provided
    echo Usage: %~nx0 ^<gma_file^>
    pause
    exit /b 1
)

set password=M3thu3n

%GM_HOME%\bin\drive_tools_archiver.exe -ei %1 "%~n1" %password%