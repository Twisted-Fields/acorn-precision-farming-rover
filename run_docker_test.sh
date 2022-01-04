docker-compose -f docker-compose-test.yml up --remove-orphans -d
docker exec -it acorn_vehicle sh -c 'coverage run -m pytest --log-cli-level DEBUG && coverage report --skip-covered --skip-empty'
