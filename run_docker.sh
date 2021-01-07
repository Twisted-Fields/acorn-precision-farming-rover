docker kill acornserver_docker
docker rm acornserver_docker
# sudo docker run --device /dev/gpiomem -td --privileged --name acornserver_docker --mount type=bind,source="$(pwd)",target=/acorn  --mount type=bind,source=/sys/firmware/devicetree/base,target=/sys/firmware/devicetree/base,readonly -p 80:80 -p 5570:5570 -p 6379:6379 --restart=unless-stopped -w /acorn/server acornserver:1.0
sudo docker run -td --privileged --name acornserver_docker -v /etc/timezone:/etc/timezone:ro -v /etc/localtime:/etc/localtime:ro --mount type=bind,source="$(pwd)",target=/acorn -p 80:80 -p 5570:5570 -p 6379:6379 --restart=unless-stopped -w /acorn/server acornserver:1.0


#/proc/device-tree/system/linux,revision
