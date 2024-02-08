#!/bin/bash

sudo true # non-op just to make sure we have root

if [ $# -eq 0 ]; then
 echo "Error: No arguments. First argument must be desired hostname."
 exit 1
fi

DIR="/media/$USER/bootfs"

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

sudo touch /media/$USER/bootfs/ssh
# sudo cp raspberry/wpa_supplicant.conf /media/$USER/bootfs/
sudo cp overlay/dt-blob.bin /media/$USER/bootfs/
sudo cp overlay/config.txt /media/$USER/bootfs/
sudo cp overlay/dual-sc16is752-spi1.dtbo /media/$USER/bootfs/overlays/
sudo cp raspberry/99-gps.rules /media/$USER/rootfs/etc/udev/rules.d/
sudo cp raspberry/80-can.network /media/$USER/rootfs/etc/systemd/network/
sudo cp raspberry/*.nmconnection /media/$USER/rootfs/etc/NetworkManager/system-connections
sudo chmod -R 600 /media/$USER/rootfs/etc/NetworkManager/system-connections/twisted*.nmconnection
sudo chown -R root:root /media/$USER/rootfs/etc/NetworkManager/system-connections/*
sudo cp raspberry/can-isotp.conf /media/$USER/rootfs/etc/modprobe.d/
sudo cp raspberry/modules /media/$USER/rootfs/etc/

# user acorn password acorn
sudo echo 'acorn:$6$kd42OyeOBbMXuLqd$slFEIHfA7g0yrs8ae4gvVUZcj38Mm2Q/kksW.jmW9w/K.eva4QHlMKsQtSWpe497MxXe4byzPwLnpubNXitSn1' > /media/$USER/bootfs/userconf

# rm -rf /media/$USER/rootfs/home/acorn/Bookshelf
# rm -rf /media/$USER/rootfs/home/acorn/Desktop
# rm -rf /media/$USER/rootfs/home/acorn/Documents
# rm -rf /media/$USER/rootfs/home/acorn/Downloads
# rm -rf /media/$USER/rootfs/home/acorn/Music
# rm -rf /media/$USER/rootfs/home/acorn/Pictures
# rm -rf /media/$USER/rootfs/home/acorn/Public
# rm -rf /media/$USER/rootfs/home/acorn/Templates
# rm -rf /media/$USER/rootfs/home/acorn/Videos

sudo mkdir -p /media/$USER/rootfs/home/acorn/acorn
sudo chown -R 1000:1000 /media/$USER/rootfs/home/acorn

#rsync -aP ../. /media/$USER/rootfs/home/acorn/acorn/
ls /media/$USER/rootfs/home/acorn

sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hosts
sudo sed -i "s/raspberrypi/$1/" /media/$USER/rootfs/etc/hostname

sync

sudo umount /media/$USER/rootfs
sudo umount /media/$USER/bootfs

echo "Completed. Now boot the pi, expand the filesystem, and then run rsync -aP ../. acorn@$1.local:/home/acorn/acorn/"
echo "Then run the post-flash script from inside the pi."
