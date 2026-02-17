@echo off

rem This script calls a python script that interrogates the current platform's
rem network adapters and based on the adapters returned determines the platform
rem the drives are running on. It then updates the config files and network
rem adapters to reflect the platform the drives are running on. Then it calls the
rem script to stop and restart the services so the changes to the config files
rem are read by the services.
rem
rem Platforms supported by this script: EGPS v1, EGPS v2, EHUB v5, EHUB v7

rem Make sure we have adminstrator privileges
NET FILE 1>NUL 2>NUL
if '%errorlevel%' == '0' (goto gotPrivileges
) else (powershell "saps -filepath %0 -verb runas" 2>&1)
exit /b

:gotPrivileges

rem Set the current drive and directory to the location of this script
cd /D %~d0
cd %~p0

echo Setting up the config files and network settings based on network hardware on the platform...
call python update_settings_for_platform.py

rem Restart the services
echo Services will be restarted to read the changes from the updated config files...
pause
%GM_HOME%\scripts\platform\gps\Switch_to_client.bat