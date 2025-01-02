@echo off
if not exist "build" mkdir "build"

cd build
cmake ../CMakeLists.txt
cmake --build ./ --config Release --target SerialPortTool

pause