echo
echo "Setup"
echo 

mkdir -p out
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=/home/andrea/UberTC-4.9/bin/arm-eabi-

echo
echo "Clean up"
echo 

make O=out clean
make O=out mrproper

echo
echo "build"
echo 

make O=out tegra_s8515_android_defconfig
make O=out -j$(nproc --all)
