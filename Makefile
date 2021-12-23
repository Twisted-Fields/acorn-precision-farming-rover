LOCAL_IMAGE = acorn_docker:1.0
REMOTE_IMAGE = merlinran/acorn_docker
ACORN_NAMES ?= simulation-1

.PHONY: list
list:
	@echo "Welcome! You may want to try one of the following:\n---"; \
	grep "^.PHONY:" Makefile | grep "#" | cut -d ":" -f 2- | sed "s/\w*#/\t#/g" | sed "s/^/make/"

.PHONY: simulation # Run the vehicle and server containers in simulation mode. Use envvar ACORN_NAMES to set the name of each simulated vehicle.
simulation: docker-image
	@docker-compose -f docker-compose-server.yml up -d && \
	for name in $(ACORN_NAMES); do \
		NAME=$${name} envsubst < docker-compose-simulation.yml | COMPOSE_IGNORE_ORPHANS=true docker-compose --file /dev/stdin --project-directory . up -d; \
	done &&\
	echo "Please visit http://localhost"

.PHONY: attach-vehicle # Attach to the shell of the first vehicle container. It creates the container if it doesn't exist.
attach-vehicle:
	@first_container=`docker ps --filter "name=acorn_*" -q | head -1`; \
	if [[ -z "$${first_container}" ]]; then \
		make docker-vehicle; \
	fi; \
	docker exec -it $${first_container} /bin/sh

.PHONY: attach-server # Attach to the shell of the server container. It creates the container if it doesn't exist.
attach-server:
	@test -z `docker ps -f "name=acorn_server" -q` && make docker-server; \
	docker exec -it acorn_server /bin/sh

.PHONY: stop # Stop all the vehicle and server containers.
stop:
	@containers=`docker ps --filter "name=acorn_*" -q`; \
	if [[ -n "$${containers}" ]]; then \
		docker stop $${containers} && docker rm $${containers}; \
	fi \

.PHONY: docker-test # Start the vehicle container in test mode and run the tests in it.
docker-test: docker-image
	@docker-compose -f docker-compose-test.yml up --remove-orphans -d && \
	docker exec -it acorn_vehicle make test

.PHONY: docker-test-watch # Keeps running tests for any changed files. Useful when developing.
docker-test-watch: docker-image
	@docker-compose -f docker-compose-test.yml up --remove-orphans -d && \
	docker exec -it acorn_vehicle ptw --poll

.PHONY: push-image # Build and push image to Docker Hub to be used by CI.
push-image: docker-image
ifeq ($(shell uname -m),arm64)
	docker buildx build -t $(REMOTE_IMAGE) --platform linux/amd64,linux/arm64 --push .
else
	docker build -t $(REMOTE_IMAGE) . && docker push $(REMOTE_IMAGE)
endif

.PHONY: test  # Run tests on Linux (if the Python dependencies are installed) or inside Docker. Otherwise, you probably want to try `make docker-test` instead.
test:
	coverage run -m pytest && coverage report --skip-covered --skip-empty

.PHONY: docker-vehicle
docker-vehicle: docker-image
	@DOCKER_COMPOSE_FILE=docker-compose-vehicle.yml make restart-docker-compose

.PHONY: docker-server
docker-server: docker-image
	@DOCKER_COMPOSE_FILE=docker-compose-server.yml make restart-docker-compose

.PHONY: restart-docker-compose
restart-docker-compose:
	@docker-compose -f $${DOCKER_COMPOSE_FILE} down --remove-orphans
	@COMPOSE_IGNORE_ORPHANS=true docker-compose -f $${DOCKER_COMPOSE_FILE} up -d

.PHONY: docker-image
docker-image: Dockerfile
	image_time=`docker images $(LOCAL_IMAGE) --format "{{.CreatedAt}}"`; \
	test -z `find Dockerfile -newermt "$${image_time}"` || docker build -t $(LOCAL_IMAGE) .

Dockerfile: vehicle/requirements.txt server/requirements.txt
	@docker build -t $(LOCAL_IMAGE) . && touch Dockerfile # Rebuilding image if requirements files have been changed
