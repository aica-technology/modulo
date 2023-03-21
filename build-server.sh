#!/bin/bash
BASE_TAG=humble-devel

IMAGE_NAME=aica-technology/modulo
IMAGE_TAG=latest

REMOTE_SSH_PORT=4440
SERVE_REMOTE=false

HELP_MESSAGE="Usage: build.sh [-p] [-r]
Options:
  -d, --development      Only target the dependencies layer to prevent
                         sources from being built or tested

  -c, --core             Only target the modulo core layer to prevent
                         modulo components from being built or tested

  -r, --rebuild          Rebuild the image using the docker
                         --no-cache option

  -v, --verbose          Use the verbose option during the building
                         process

  -s, --serve            Start the remove development server
"

BUILD_FLAGS=(--build-arg BASE_TAG="${BASE_TAG}")
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -d|--development) BUILD_FLAGS+=(--target dependencies) ; IMAGE_TAG=development ; shift ;;
    -c|--core) BUILD_FLAGS+=(--target modulo-core) ; IMAGE_TAG=development ; shift ;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -s|--serve) SERVE_REMOTE=true ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

docker pull ghcr.io/aica-technology/ros2-control-libraries:"${BASE_TAG}"
DOCKER_BUILDKIT=1 docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" "${BUILD_FLAGS[@]}" . || exit 1

if [[ "${SERVE_REMOTE}" == true ]]; then
  aica-docker server "${IMAGE_NAME}:${IMAGE_TAG}" -u ros2 -p "${REMOTE_SSH_PORT}"
fi