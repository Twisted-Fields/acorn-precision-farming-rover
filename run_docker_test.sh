docker-compose -f docker-compose-test.yml up --remove-orphans -d
docker exec -it acorn_vehicle pytest --log-cli-level DEBUG
