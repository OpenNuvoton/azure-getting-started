:: Copyright (c) Microsoft Corporation.
:: Licensed under the MIT License.

@echo off

echo Installing prerequisites. Please leave the window open until the installation completes.

net session >nul 2>&1
if NOT %errorLevel% == 0 (
    echo.
    echo Error: Unable to install, please execute bat file with Administrator privilages.
    echo.
    goto finish
) 

echo.

set cmake_path=https://github.com/Kitware/CMake/releases/download/v3.18.0/
set cmake_file=cmake-3.18.0-win32-x86.msi

set gccarm_path=https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/
set gccarm_file=gcc-arm-none-eabi-9-2019-q4-major-win32-sha2.exe

set ninja_path=https://github.com/ninja-build/ninja/releases/download/v1.10.0/
set ninja_file=ninja-win.zip

set termite_path=https://www.compuphase.com/software/
set termite_file=termite-3.4.exe

set iot_explorer_path=https://github.com/Azure/azure-iot-explorer/releases/download/v0.11.1/
set iot_explorer_file=Azure.IoT.Explorer.preview.0.11.1.msi

echo Downloading CMake...
powershell (New-Object Net.WebClient).DownloadFile('%cmake_path%%cmake_file%', '%TEMP%\%cmake_file%')

echo Downloading GCC-ARM...
powershell (New-Object Net.WebClient).DownloadFile('%gccarm_path%%gccarm_file%', '%TEMP%\%gccarm_file%')

echo Downloading Ninja...
powershell (New-Object Net.WebClient).DownloadFile('%ninja_path%%ninja_file%', '%TEMP%\%ninja_file%')

echo Downloading Termite...
powershell (New-Object Net.WebClient).DownloadFile('%termite_path%%termite_file%', '%TEMP%\%termite_file%')

echo Downloading IoT Explorer...
powershell (New-Object Net.WebClient).DownloadFile('%iot_explorer_path%%iot_explorer_file%', '%TEMP%\%iot_explorer_file%')

echo.

echo Installing CMake...
"%TEMP%\%cmake_file%" ADD_CMAKE_TO_PATH=System /passive

echo Installing GCC-ARM...
"%TEMP%\%gccarm_file%" /S /P /R

echo Installing Ninja...
if not exist "%ProgramFiles(x86)%\ninja" mkdir "%ProgramFiles(x86)%\ninja"
"%~dp0\pathman.exe" /as "%ProgramFiles(x86)%\ninja"
powershell Expand-Archive -Force -Path '%TEMP%\%ninja_file%' -DestinationPath '%ProgramFiles(x86)%\ninja'

echo Installing Termite...
"%TEMP%\%termite_file%" /S

echo Installing IoT Explorer...
"%TEMP%\%iot_explorer_file%" /passive

echo.
echo Installation complete! Successfully installed CMake, GCC-ARM, Ninja and Termite.
echo.

:finish
pause
