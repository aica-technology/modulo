#!/bin/bash

IMAGE_NAME=ghcr.io/aica-technology/modulo
IMAGE_TAG=latest

ROS2_VERSION=humble
CL_VERSION=v7.1.1

SSH_PORT=4440

HELP_MESSAGE="Usage: build-server.sh [options]
Options:
  --test                   Target the test layer to run the tests.

  -s|--serve               Start the remote development server on port $SSH_PORT.

  --cl-version <VERSION>   Specify the version of the control libraries image to use.
                           (default: $CL_VERSION)

  -v|--verbose             Set the build output to verbose.

  --cache-id <id>          Invalidate the mount cache (e.g. CMake build folder)
                           by providing a new value.

  -r|--no-cache            Invalidate all cache (layer + mount).

  -h|--help                Show this help message.
"

TEST=0
SERVE_REMOTE=0
BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --test) BUILD_FLAGS+=(--target=test); TEST=1; IMAGE_TAG=test; shift 1;;
    -s|--serve) BUILD_FLAGS+=(--target build); SERVE_REMOTE=1; IMAGE_TAG=build; shift 1;;
    --cl-version) CL_VERSION=$2; shift 2;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift 1;;
    --cache-id) BUILD_FLAGS+=(--build-arg CACHEID=$2); shift 2;;
    -r|--no-cache) BUILD_FLAGS+=(--no-cache); BUILD_FLAGS+=(--build-arg CACHEID=$(date +%s)); shift 1;;
    -h|--help) echo "$HELP_MESSAGE"; exit 0;;
    -*) echo "Unknown option: $1" >&2; echo "$HELP_MESSAGE"; exit 1;;
  esac
done

if [ "$((TEST + SERVE_REMOTE))" -gt 1 ]; then
  echo "Error in command line arguments:" >&2
  echo "--test and --serve options are mutually exclusive." >&2
  exit 1
fi

BUILD_FLAGS+=(--build-arg CL_VERSION="${CL_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1

if [ "${SERVE_REMOTE}" -eq 1 ]; then
  aica-docker server "${IMAGE_NAME}:${IMAGE_TAG}" --user ros2 --port ${SSH_PORT}
fi
