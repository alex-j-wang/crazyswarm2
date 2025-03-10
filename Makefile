# Set default variables
DOCKER_REPOSITORY := crazyswarm2
DOCKER_TAG := latest
DOCKER_IMAGE := $(DOCKER_REPOSITORY):$(DOCKER_TAG)
DOCKERFILE := Dockerfile
CURRENT_DIR := $(shell pwd)
ROS_WS_PATH := /ros_ws/src
WORKERS := $(shell expr `sysctl -n hw.physicalcpu` - 2)
ROS_SOURCE_CMD := source /opt/ros/humble/setup.bash

.PHONY: check-image build run shell clean

# Start the container
run: check-image
	@docker run all --rm -it \
		--volume $(CURRENT_DIR):$(ROS_WS_PATH):rw \
		--name $(DOCKER_REPOSITORY) \
		$(DOCKER_IMAGE) bash

# Look for dev container, build if it doesn't exist
check-image:
	@if ! docker images -q $(DOCKER_IMAGE) > /dev/null; then \
		echo "Image not found, building..."; \
		docker build -t $(DOCKER_IMAGE) -f $(DOCKERFILE) .; \
	fi

# Build source files inside the running container
build:
	@echo "Building with $(WORKERS) workers"
	@docker exec -it $(DOCKER_REPOSITORY) bash -c "$(ROS_SOURCE_CMD) && cd /ros_ws && colcon build --symlink-install --parallel-workers $(WORKERS)"
	@docker exec -it $(DOCKER_REPOSITORY) bash -c "cd /ros_ws && colcon build --symlink-install --parallel-workers $(WORKERS)"

# Run a shell inside the running container
shell:
	@docker exec -it $(DOCKER_REPOSITORY) bash -c "$(ROS_SOURCE_CMD) && bash"

# Clean up the container
clean:
	@docker rm -f $(DOCKER_REPOSITORY) || true
	@docker rmi -f $(DOCKER_IMAGE) || true