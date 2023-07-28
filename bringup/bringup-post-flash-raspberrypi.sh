
sudo timedatectl set-timezone US/Pacific
sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd
sudo apt-get update && sudo apt-get -y upgrade
sudo apt install can-utils
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker acorn
sudo apt -y install vim libffi-dev libssl-dev python3-dev python3-pip
sudo pip3 install docker-compose crccheck
newgrp docker
echo "Now run docker-compose -f docker-compose-vehicle.yml build in the acorn directory."
