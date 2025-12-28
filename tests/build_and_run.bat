@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

:: 1. Find Visual Studio
for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    set "VS_PATH=%%i"
)

if not defined VS_PATH (
    echo Error: Visual Studio not found.
    pause
    exit /b 1
)

:: 2. Setup Environment (suppress output)
call "!VS_PATH!\VC\Auxiliary\Build\vcvars64.bat" >nul

:: 3. Compile (Simple CL command)
echo Compiling...
cl /nologo /EHsc /O2 test_throughput.cpp
if %errorlevel% neq 0 (
    echo Compilation failed.
    pause
    exit /b 1
)

:: 4. Cleanup artifacts
del *.obj

:: 5. Run
echo.
echo Running on COM6...
test_throughput.exe COM6

