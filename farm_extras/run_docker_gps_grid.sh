docker kill acorn_gps_grid
docker rm acorn_gps_grid
docker-compose -f ../docker-compose-gps-grid.yml up --remove-orphans -d
