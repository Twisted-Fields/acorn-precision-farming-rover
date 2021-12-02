
Compiling a device tree overlay
===

For additional serial ports via hardware expansion board.

https://www.raspberrypi.com/documentation/computers/configuration.html#providing-a-custom-device-tree-blob

sudo apt install device-tree-compiler

sudo dtc -I dts -O dtb -o dual-sc16is752-spi1.dtbo sc16is752-spi1-overlay_dual.dts
