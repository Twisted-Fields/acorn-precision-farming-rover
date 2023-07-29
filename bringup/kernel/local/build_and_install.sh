cd /home/acorn/linux
KERNEL=kernel8
make bcm2711_defconfig
make -j4 Image.gz modules dtbs
sudo make modules_install
sudo cp arch/arm64/boot/dts/broadcom/*.dtb /boot/
sudo cp arch/arm64/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm64/boot/dts/overlays/README /boot/overlays/
sudo cp arch/arm64/boot/Image.gz /boot/$KERNEL.img

sudo echo "
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with '#' are ignored.
can-isotp
" > /etc/modules

sudo echo "options can-isotp max_pdu_size=200000" > /etc/modprobe.d/can-isotp.conf
