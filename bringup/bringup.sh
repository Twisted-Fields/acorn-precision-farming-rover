#!/bin/bash
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
mkdir /media/$USER/rootfs/home/pi/acorn
sudo rsync -aP ../. /media/$USER/rootfs/home/pi/acorn/
sudo ls /media/$USER/rootfs/home/pi
sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hosts
sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hostname


echo "Completed"

sync

sudo umount /media/$USER/rootfs
sudo umount /media/$USER/boot
