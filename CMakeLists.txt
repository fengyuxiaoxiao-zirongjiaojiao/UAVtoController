cmake_minimum_required(VERSION 3.1)
project(uavToController)

set(CMAKE_CXX_STANDARD 11)

include_directories(libs/mavlink)
include_directories(libs/mavlink/ardupilotmega)

set(SOURCE_FILES 
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/Application.cpp
    ${PROJECT_SOURCE_DIR}/src/Comm/SerialLink.cpp
    ${PROJECT_SOURCE_DIR}/src/Protocol/ProtocolMavlink.cpp
    ${PROJECT_SOURCE_DIR}/src/Comm/UDPLink.cpp
    ${PROJECT_SOURCE_DIR}/src/Protocol/ProtocolCtrlCenter.cpp
    ${PROJECT_SOURCE_DIR}/src/Protocol/CtrlCenter.cpp
)

# SerialPortTool
set(SerialPortTool_DIR ${CMAKE_SOURCE_DIR}/libs/SerialPortTool)
list(APPEND SOURCE_FILES 
    ${SerialPortTool_DIR}/src/SerialPort.cpp 
    ${SerialPortTool_DIR}/src/SerialPortBase.cpp 
    ${SerialPortTool_DIR}/src/SerialPortInfo.cpp 
    ${SerialPortTool_DIR}/src/SerialPortInfoBase.cpp
)
if (CMAKE_HOST_WIN32)
    list(APPEND SOURCE_FILES 
        ${SerialPortTool_DIR}/src/SerialPortInfoWinBase.cpp 
        ${SerialPortTool_DIR}/src/SerialPortWinBase.cpp
    )
elseif (CMAKE_HOST_UNIX)
    list(APPEND SOURCE_FILES 
        ${SerialPortTool_DIR}/src/SerialPortInfoUnixBase.cpp
        ${SerialPortTool_DIR}/src/SerialPortUnixBase.cpp
    )
endif ()

include_directories(
    ${SerialPortTool_DIR}/include
    ${SerialPortTool_DIR}/src
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/Protocol
    ${PROJECT_SOURCE_DIR}/src/Comm
)

set(Libraries

)

if (CMAKE_HOST_UNIX)
    list(APPEND Libraries
        pthread
    )
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${Libraries})