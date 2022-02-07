#!/bin/bash

TOOLCHAIN64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"

mkdir -p build64
cd build64
cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -std=c++11 -march=armv8-a -L  /usr/aarch64-linux-gnu-2.23/lib -I  /usr/aarch64-linux-gnu-2.23/include" ../
make -j4
cd ../
