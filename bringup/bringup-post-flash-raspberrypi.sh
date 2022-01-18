sudo apt-get update && sudo apt-get upgrade
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker pi
sudo apt install vim libffi-dev libssl-dev python3-dev python3-pip
sudo pip3 install docker-compose
newgrp docker
echo "Now run docker-compose -f docker-compose-vehicle.yml build in the acorn directory."
