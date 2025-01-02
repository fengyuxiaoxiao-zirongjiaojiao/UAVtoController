cmake_minimum_required(VERSION 3.1)
project(SerialPortTool)

# 设置为1则表示交叉编译，设置为0则表示x86 gcc编译
SET(CROSS_COMPILE 1)

# c++11标准
set(CMAKE_CXX_STANDARD 11)


if(CMAKE_HOST_UNIX)
  message("building in Linux system.")
elseif(CMAKE_HOST_WIN32)
  # 忽略这警告
  add_compile_options(/wd4018 /wd4819 /wd4244 /wd4305 /wd4805)
  set(CROSS_COMPILE 0)
  message("building in windows system.")
else()
  message(FATAL_ERROR "unsupport system. " ${CMAKE_HOST_SYSTEM_NAME})
endif()

IF(CROSS_COMPILE)
set(SWAGGER_API 0)
set(TEST_COMPILE 0)
SET(CMAKE_SYSTEM_NAME Linux)
# SET(TOOLCHAIN_DIR "/opt/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/")
# set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-linux-gnu-g++)
# set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/aarch64-linux-gnu-gcc)
set(TOOLCHAIN_DIR "/opt/arm-AX620E-linux-uclibcgnueabihf/")
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/arm-AX620E-linux-uclibcgnueabihf-g++)
set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/arm-AX620E-linux-uclibcgnueabihf-gcc) 

#set(GNU_FLAGS "-mfpu=vfp -fPIC")
#set(CMAKE_CXX_FLAGS "${GNU_FLAGS} ")
#set(CMAKE_C_FLAGS "${GNU_FLAGS}  ")

SET(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR})
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

ENDIF(CROSS_COMPILE)



# set(HEADER_FILES
#     include/
# )

# set(SOURCE_FILES
#     src/main.cpp

# )


list(APPEND SOURCE_FILES src/main.cpp src/SerialPort.cpp src/SerialPortBase.cpp src/SerialPortInfo.cpp src/SerialPortInfoBase.cpp)
if (CMAKE_HOST_WIN32)
    list(APPEND SOURCE_FILES src/SerialPortInfoWinBase.cpp src/SerialPortWinBase.cpp)
elseif (CMAKE_HOST_UNIX)
    list(APPEND SOURCE_FILES src/SerialPortInfoUnixBase.cpp src/SerialPortUnixBase.cpp)
endif ()


# 宏定义
add_definitions( 
  -DVERSION="1.0"
  -DAUTH="build by vincent_xjw@163.com"
)

include_directories(
  src
  include
)
message("VERSION: '${VERSION}'")
message("AUTH: '${AUTH}'")



if (CMAKE_HOST_WIN32)
    # for function availableFriendlyPorts
    SET(Libraries setupapi)
elseif(CMAKE_HOST_UNIX)
    SET(Libraries pthread)
endif ()

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

# 将目标文件与库文件进行链接
target_link_libraries(${PROJECT_NAME} ${Libraries})
