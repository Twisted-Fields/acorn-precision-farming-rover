docker kill acorn_server
docker rm acorn_server
docker-compose -f docker-compose-server.yml up --remove-orphans -d
