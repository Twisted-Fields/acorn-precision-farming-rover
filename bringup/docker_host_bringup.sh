sudo apt-get update && sudo apt-get upgrade
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker pi
sudo apt install vim libffi-dev libssl-dev python3-dev python3-pip
sudo pip3 install docker-compose
sudo bash -c "echo dtoverlay=dual-sc16is752-spi1 >> /boot/config.txt"
sudo cp overlay/dual-sc16is752-spi1.dtbo /boot/overlays/
