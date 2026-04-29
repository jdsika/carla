@echo off
setlocal EnableDelayedExpansion

rem --------------------------------------------------------------------------
rem CARLA — Phase A: Build and install OSI dependencies (Windows)
rem
rem Builds asam-osi-utilities (and its transitive deps: protobuf, lz4, zstd)
rem into a self-contained install prefix that Phase B (CARLA CMake configure)
rem consumes via find_package().
rem
rem Usage:
rem   scripts\build-osi-deps.bat [--config Release|Debug] [--prefix <path>]
rem                               [--triplet <vcpkg-triplet>] [--clean]
rem
rem Environment:
rem   VCPKG_ROOT — path to vcpkg checkout (required)
rem --------------------------------------------------------------------------

set SCRIPT_DIR=%~dp0
set CARLA_ROOT=%SCRIPT_DIR%..
set OSI_SRC=%CARLA_ROOT%\externals\asam-osi-utilities

rem Defaults
set CONFIG=Release
set PREFIX=%CARLA_ROOT%\Build\osi-deps
set BUILD_DIR=%CARLA_ROOT%\Build\osi-build
set TRIPLET=x64-windows-static-md
set CLEAN=0

rem -- PARSE ARGUMENTS --
:parse
if "%1"=="" goto validate
if "%1"=="--config" (
    set CONFIG=%2
    shift
    shift
    goto parse
)
if "%1"=="--prefix" (
    set PREFIX=%2
    shift
    shift
    goto parse
)
if "%1"=="--triplet" (
    set TRIPLET=%2
    shift
    shift
    goto parse
)
if "%1"=="--clean" (
    set CLEAN=1
    shift
    goto parse
)
if "%1"=="-h" goto usage
if "%1"=="--help" goto usage
echo Unknown option: %1
exit /b 1

:usage
echo Usage: %~nx0 [--config Release^|Debug] [--prefix ^<path^>] [--triplet ^<triplet^>] [--clean]
exit /b 0

rem -- VALIDATE --
:validate
if not exist "%OSI_SRC%\CMakeLists.txt" (
    echo ERROR: asam-osi-utilities submodule not found at %OSI_SRC%
    echo Run:  git submodule update --init --recursive externals\asam-osi-utilities
    exit /b 1
)

if "%VCPKG_ROOT%"=="" (
    echo ERROR: VCPKG_ROOT is not set. Set it to your vcpkg installation.
    echo   e.g.: set VCPKG_ROOT=C:\vcpkg
    exit /b 1
)

if not exist "%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" (
    echo ERROR: vcpkg toolchain not found at %VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
    exit /b 1
)

echo ========================================
echo   CARLA OSI Dependencies — Phase A
echo ========================================
echo   Source:   %OSI_SRC%
echo   Build:    %BUILD_DIR%
echo   Prefix:   %PREFIX%
echo   Config:   %CONFIG%
echo   Triplet:  %TRIPLET%
echo   VCPKG:    %VCPKG_ROOT%
echo ========================================

rem -- CLEAN --
if %CLEAN%==1 (
    echo Cleaning previous build...
    if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
    if exist "%PREFIX%" rmdir /s /q "%PREFIX%"
)

rem -- CONFIGURE --
echo.
echo ^>^>^> Configuring asam-osi-utilities...

cmake -S "%OSI_SRC%" ^
      -B "%BUILD_DIR%" ^
      -G Ninja ^
      -DCMAKE_BUILD_TYPE=%CONFIG% ^
      -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake ^
      -DVCPKG_TARGET_TRIPLET=%TRIPLET% ^
      -DBUILD_EXAMPLES=OFF ^
      -DBUILD_TESTING=OFF ^
      -DBUILD_DOCS=OFF
if errorlevel 1 (
    echo ERROR: CMake configure failed
    exit /b 1
)

rem -- BUILD --
echo.
echo ^>^>^> Building asam-osi-utilities...

cmake --build "%BUILD_DIR%" --config %CONFIG% --parallel
if errorlevel 1 (
    echo ERROR: CMake build failed
    exit /b 1
)

rem -- INSTALL --
echo.
echo ^>^>^> Installing to %PREFIX%...

cmake --install "%BUILD_DIR%" --config %CONFIG% --prefix "%PREFIX%"
if errorlevel 1 (
    echo ERROR: CMake install failed
    exit /b 1
)

rem -- SUMMARY --
set VCPKG_INSTALLED=%BUILD_DIR%\vcpkg_installed\%TRIPLET%

echo.
echo ========================================
echo   Phase A complete!
echo ========================================
echo.
echo To build CARLA with OSI support (Phase B):
echo.
echo   cmake -G Ninja -S . -B Build ^
echo     --toolchain CMake\Toolchain.cmake ^
echo     -DCMAKE_BUILD_TYPE=%CONFIG% ^
echo     -DENABLE_OSI=ON ^
echo     -DCARLA_OSI_PREFIX=%PREFIX% ^
echo     -DCARLA_OSI_VCPKG_INSTALLED=%VCPKG_INSTALLED%
echo.

endlocal
