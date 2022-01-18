
Compiling a device tree overlay
===

For additional serial ports via hardware expansion board.

https://www.raspberrypi.com/documentation/computers/configuration.html#providing-a-custom-device-tree-blob

sudo apt install device-tree-compiler

Serial Overlay:
sudo dtc -I dts -O dtb -o triple-sc16is752-spi1.dtbo sc16is752-spi1-overlay_triple.dts
sudo cp triple-sc16is752-spi1.dtbo /boot/overlay

GPIO Overlay:
sudo dtc -I dts -O dtb -o /boot/dt-blob.bin dt-blob-dualcam.dts


config.txt:
dtparam=spi=on
dtoverlay=triple-sc16is752-spi1
dtparam=i2c1=on
dtparam=i2c3=on
dtoverlay=uart3
dtoverlay=uart0 #
dtoverlay=spi0 # e-paper display (not working)
console=ttyAMA0,115200
