#!/usr/bin/env bash

set -e

DOCKER_REPOSITORY="crazyswarm2"
DOCKER_TAG="latest"
DOCKER_IMAGE="${DOCKER_REPOSITORY}:${DOCKER_TAG}"
DOCKERFILE="Dockerfile"

if [ -z "$(docker images -q ${DOCKER_IMAGE})" ]; then
  echo "Image not found, building..."
  docker build -t "${DOCKER_IMAGE}" -f "${DOCKERFILE}" .
else
  echo "Image already exists."
fi