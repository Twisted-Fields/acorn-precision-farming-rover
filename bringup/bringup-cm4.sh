#!/bin/bash

sudo true # non-op just to make sure we have root

if [ $# -eq 0 ]; then
 echo "Error: No arguments. First argument must be desired hostname."
 exit 1
fi

DIR="/media/$USER/boot"

if [ ! -d "$DIR" ]; then
  # script statements if $DIR doesn't exist.
  sudo rpiboot
  sleep 10
fi

if [ ! -d "$DIR" ]; then
  # script statements if $DIR doesn't exist.
  echo "Failed. Directory doesn't exist."
  exit 1
fi

sudo touch /media/$USER/boot/ssh
sudo cp raspberry/wpa_supplicant.conf /media/$USER/boot/
sudo cp overlay/dt-blob.bin /media/$USER/boot/
sudo cp overlay/config.txt /media/$USER/boot/
sudo cp overlay/dual-sc16is752-spi1.dtbo /media/$USER/boot/overlays/
sudo cp raspberry/99-gps.rules /media/$USER/rootfs/etc/udev/rules.d/
sudo cp raspberry/80-can.network /media/$USER/etc/systemd/network/

# user acorn password acorn
sudo echo 'acorn:$6$kd42OyeOBbMXuLqd$slFEIHfA7g0yrs8ae4gvVUZcj38Mm2Q/kksW.jmW9w/K.eva4QHlMKsQtSWpe497MxXe4byzPwLnpubNXitSn1' > /media/$USER/boot/userconf

# rm -rf /media/$USER/rootfs/home/acorn/Bookshelf
# rm -rf /media/$USER/rootfs/home/acorn/Desktop
# rm -rf /media/$USER/rootfs/home/acorn/Documents
# rm -rf /media/$USER/rootfs/home/acorn/Downloads
# rm -rf /media/$USER/rootfs/home/acorn/Music
# rm -rf /media/$USER/rootfs/home/acorn/Pictures
# rm -rf /media/$USER/rootfs/home/acorn/Public
# rm -rf /media/$USER/rootfs/home/acorn/Templates
# rm -rf /media/$USER/rootfs/home/acorn/Videos

sudo mkdir /media/$USER/rootfs/home/acorn
sudo chown taylor:taylor /media/$USER/rootfs/home/acorn
mkdir /media/$USER/rootfs/home/acorn/acorn
#rsync -aP ../. /media/$USER/rootfs/home/acorn/acorn/
ls /media/$USER/rootfs/home/acorn

sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hosts
sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hostname

sync

sudo umount /media/$USER/rootfs
sudo umount /media/$USER/boot

echo "Completed"
