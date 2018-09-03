export ARCH=arm && export SUBARCH=arm

make clean && make mrproper

export CROSS_COMPILE=/home/andrea/U4/bin/arm-eabi-

make tegra_s8515_android_defconfig

make -j2

