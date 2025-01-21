@echo off

rem Make sure we have adminstrator privileges
NET FILE 1>NUL 2>NUL
if '%errorlevel%' == '0' (goto gotPrivileges
) else (powershell "saps -filepath %0 -verb runas" 2>&1)
exit /b

:gotPrivileges

%~d0
cd %~p0

echo Setting up the config files and network adapters...
call python update_config_and_adapters.py > output.log

rem Restart the services
echo Services will be restarted to read the changes from the updated config files...
pause
c:\epic\scripts\platform\gps\Switch_to_client.bat