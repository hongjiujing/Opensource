#!/bin/bash
sudo ./arm-softmmu/qemu-system-arm -M beaglexm -m 512 -clock unix -serial stdio -sd ~/workbench/code/linaro/beagle_sd.img
#./configure --target-list=arm-softmmu --enable-sdl --enable-skinning
#sudo ./arm-softmmu/qemu-system-arm -M n900 -mtdblock ~/download/RX-51_2009SE_20.2010.36-2.nand -sd ~/download/RX-51_2009SE_20.2010.36-2.emmc -serial stdio -clock unix  -skin ./skin/n900/skin.xml
