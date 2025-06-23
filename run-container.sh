#!/usr/bin/env bash

set -e

DOCKER_REPOSITORY="crazyswarm2"
DOCKER_TAG="latest"
DOCKER_IMAGE="${DOCKER_REPOSITORY}:${DOCKER_TAG}"
CURRENT_DIR="$(pwd)"
ROS_WS_PATH="/ros_ws/src"

if [ "$1" == "--clean" ]; then
  echo "Removing existing container..."
  docker container rm "${DOCKER_REPOSITORY}" || true
fi

docker run -it --privileged -d \
  --volume "${CURRENT_DIR}:${ROS_WS_PATH}:rw" \
  --volume /dev/bus/usb:/dev/bus/usb \
  --name "${DOCKER_REPOSITORY}" \
  --network host \
  --shm-size=1000mb \
  "${DOCKER_IMAGE}" bash