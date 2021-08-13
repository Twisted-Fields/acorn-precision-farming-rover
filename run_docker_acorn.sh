docker kill acorn_vehicle
docker rm acorn_vehicle
docker-compose -f docker-compose-vehicle.yml up --remove-orphans -d
