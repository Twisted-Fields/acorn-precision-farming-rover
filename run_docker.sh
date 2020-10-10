sudo docker run -td --mount type=bind,source="$(pwd)",target=/acorn -p 80:80 -p 5570:5570 -p 6379:6379 --restart=unless-stopped -w /acorn/server acornserver:1.0
