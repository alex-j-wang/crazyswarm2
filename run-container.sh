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
  --env DISPLAY=$DISPLAY \
  --env RCUTILS_COLORIZED_OUTPUT=1 \
  --env PYTHONPATH="/ros_ws/src/crazyflie_mpc/lib:$PYTHONPATH" \
  --volume "${CURRENT_DIR}:${ROS_WS_PATH}:rw" \
  --volume /dev/bus/usb:/dev/bus/usb \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name "${DOCKER_REPOSITORY}" \
  --network host \
  --shm-size=1000mb \
  --device /dev/dri \
  "${DOCKER_IMAGE}" bash
