@echo off
call "C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat" >nul
cl /nologo /TC /W3 /Fe:vision_regression.exe vision_regression.c
if errorlevel 1 exit /b 1
vision_regression.exe
