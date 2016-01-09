export ARCH=arm && export SUBARCH=arm

export CROSS_COMPILE=/home/andrea/U4.8/bin/arm-eabi-

make tegra_s8515_android_defconfig

make -j2

