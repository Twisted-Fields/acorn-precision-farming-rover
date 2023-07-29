FOLDER=/home/taylor/Software/kernel_acorn
cd $FOLDER
KERNEL=kernel8
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig -j 24
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs -j 24
