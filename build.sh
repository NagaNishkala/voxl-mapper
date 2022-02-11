#!/bin/bash

## voxl-cross contains the following toolchains
## first two for 820, last for 865
TOOLCHAIN32="/opt/cross_toolchain/arm-gnueabi-4.9.toolchain.cmake"
TOOLCHAIN64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"
TOOLCHAIN865="/opt/cross_toolchain/aarch64-gnu-7.toolchain.cmake"

# placeholder in case more cmake opts need to be added later
EXTRA_OPTS=""

# mode variable set by arguments
MODE=""

print_usage(){
	echo ""
	echo " Build the current project in one of these modes based on build environment."
	echo ""
	echo " Usage:"
	echo ""
	echo "  ./build.sh 820"
	echo "        Build 64-bit binaries for 820"
	echo ""
	echo "  ./build.sh 865"
	echo "        Build 64-bit binaries for 865"
	echo ""
	echo "  ./build.sh native"
	echo "        Build with the native gcc/g++ compilers."
	echo ""
	echo ""
}



MODE="$1"

case "$MODE" in
	820)
		mkdir -p build64
		cd build64
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -std=c++11 -march=armv8-a -L  /usr/aarch64-linux-gnu-2.23/lib -I  /usr/aarch64-linux-gnu-2.23/include" ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;
	865)
		mkdir -p build
		cd build
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN865} ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;

	native)
		mkdir -p build
		cd build
		cmake ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;

	*)
		print_usage
		exit 1
		;;
esac



