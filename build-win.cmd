@echo off

setlocal enabledelayedexpansion

set version="v1.0"
set "currentPath=%~dp0"


if not exist "build" mkdir "build"

set buildType=Release

cd build
cmake ../CMakeLists.txt -DCMAKE_BUILD_TYPE=%buildType%
cmake --build ./ --config %buildType% --target UAVtoController


