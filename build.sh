#!/bin/bash

TOOLCHAIN64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"

mkdir -p build64
cd build64
cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} ../
make -j4
cd ../
