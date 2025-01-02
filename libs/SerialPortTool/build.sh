#!/bin/sh
###
 # @Description: 
 # @Author: vincent_xjw@163.com
 # @Date: 2023-03-03 10:30:07
 # @LastEditTime: 2023-11-14 14:47:14
 # @LastEditors: vincent_xjw@163.com
### 
if [ ! -d "build" ]; then
    echo "not exist build dir, create it."
    mkdir -p build
else
    echo "exist build dir."
fi

cd build
echo `pwd`
cmake ../CMakeLists.txt --build ./ --open ../
make -j8
