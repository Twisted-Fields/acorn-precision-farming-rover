#!/bin/bash

sudo true # non-op just to make sure we have root

if [ $# -eq 0 ]; then
 echo "no arguments"
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
sudo cp overlay/triple-sc16is752-spi1.dtbo /media/$USER/boot/overlay
sudo cp raspberry/99-gps.rules /media/$USER/rootfs/etc/udev/rules.d/

mkdir /media/$USER/rootfs/home/pi/acorn
sudo rsync -aP ../. /media/$USER/rootfs/home/pi/acorn/
sudo ls /media/$USER/rootfs/home/pi

sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hosts
sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hostname

# TODO: Copy GPS rules


echo "Completed"

sync

sudo umount /media/$USER/rootfs
sudo umount /media/$USER/boot