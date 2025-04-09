UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Linux)
	WORKERS := $(shell expr `nproc` - 2)
	DOCKER_RUN_DETACH := --rm
else ifeq ($(UNAME_S),Darwin)
	WORKERS := $(shell expr `sysctl -n hw.ncpu` - 2)
	DOCKER_RUN_DETACH := -d
else
	WORKERS := 1
	DOCKER_RUN_DETACH := -d
endif

# Set default variables
DOCKER_REPOSITORY := crazyswarm2
DOCKER_TAG := latest
DOCKER_IMAGE := $(DOCKER_REPOSITORY):$(DOCKER_TAG)
DOCKERFILE := Dockerfile
CURRENT_DIR := $(shell pwd)
ROS_WS_PATH := /ros_ws/src
ROS_SOURCE_CMD := source /opt/ros/humble/setup.sh

.PHONY: run build shell clean

# Start the container
run: check-image
	@docker run -it --privileged $(DOCKER_RUN_DETACH) \
		--volume "$(CURRENT_DIR):$(ROS_WS_PATH):rw" \
		--volume /dev/bus/usb:/dev/bus/usb \
		--name $(DOCKER_REPOSITORY) \
		$(DOCKER_IMAGE) bash

# Look for dev container, build if it doesn't exist
check-image:
	@if [ -z "$$(docker images -q $(DOCKER_IMAGE))" ]; then \
		echo "Image not found, building..."; \
		docker build -t $(DOCKER_IMAGE) -f $(DOCKERFILE) .; \
	else \
		echo "Image already exists."; \
	fi

# Build source files inside the running container
build:
	@echo "Building with $(WORKERS) workers"
	@docker exec -it $(DOCKER_REPOSITORY) bash -c "$(ROS_SOURCE_CMD) && cd /ros_ws && colcon build --symlink-install --parallel-workers $(WORKERS) --cmake-args -DCMAKE_BUILD_TYPE=Debug"

# Run a shell inside the running container
shell:
	docker exec -it $(DOCKER_REPOSITORY) bash

# Clean up the container
clean:
	docker rmi -f $(DOCKER_IMAGE) || true
