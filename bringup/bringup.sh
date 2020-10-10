sudo apt update
sudo apt install python3-pip git
sudo apt install libatlas3-base
sudo bash -c "echo dtoverlay=sc16is752-merge >> /boot/config.txt"
sudo cp overlay/sc16is752-merge.dtbo /boot/overlays/
sudo pip3 install pyserial RPi.GPIO board adafruit-blinka adafruit-circuitpython-mcp230xx
git clone https://github.com/switchdoclabs/SDL_Pi_HDC1080_Python3.git


# rsync RTKLIB folder to pi
# ~/RTKLIB/app $ sudo make install
