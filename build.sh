#!/bin/sh
###
 # @Author: vincent vincent_xjw@163.com
 # @Date: 2024-12-28 10:34:30
 # @LastEditors: vincent vincent_xjw@163.com
 # @LastEditTime: 2025-01-16 18:25:31
 # @FilePath: /UAVtoController/build.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 

if [ ! -d "build" ]; then
    echo "not exist build dir, create it."
    mkdir -p build
else
    echo "exist build dir."
fi

cd build
cmake ../CMakeLists.txt -B ./ -S ../ -DCMAKE_BUILD_TYPE=Debug
make -j8 && cd ../
