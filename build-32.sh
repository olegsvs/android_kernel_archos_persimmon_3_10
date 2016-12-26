#!/bin/bash

export KBUILD_BUILD_USER=oleg.svs

export KBUILD_BUILD_HOST=SRT

echo "Make dirs >>>"

mkdir tools/tools

echo "Export toolchains >>>"

export ARCH=arm CROSS_COMPILE=../arm-eabi-4.8/bin/arm-eabi-

echo "Make defconfig >>>"

make orange_defconfig

echo "Start build >>>"

	time make -j4

echo "======================"

