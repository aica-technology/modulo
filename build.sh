#!/bin/bash

IMAGE_NAME=ghcr.io/aica-technology/modulo
IMAGE_TAG=latest

HELP_MESSAGE="Usage: build.sh [options]
Options:
  --test                   Target the test layer to run the tests.

  --cl-version <VERSION>   Specify the version of the control libraries image to use.

  --base-version <VERSION> Specify the version of base image to use.

  --tag <TAG>              Specify the tag of the generated image.
                           (default: $IMAGE_TAG)

  -v|--verbose             Set the build output to verbose.

  --cache-id <id>          Invalidate the mount cache (e.g. CMake build folder)
                           by providing a new value.

  -r|--no-cache            Invalidate cache.

  -h|--help                Show this help message.
"

BUILD_FLAGS=()
BASE_VERSION=""
CL_VERSION=""
CACHEID=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --test) BUILD_FLAGS+=(--target=test); IMAGE_TAG=test; shift 1;;
    --cl-version) CL_VERSION=$2; shift 2;;
    --base-version) BASE_VERSION=$2; shift 2;;
    --tag) IMAGE_TAG=$2; shift 2;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift 1;;
    --cache-id) CACHEID=$2; shift 2;;
    -r|--no-cache) BUILD_FLAGS+=(--no-cache); shift 1;;
    -h|--help) echo "$HELP_MESSAGE"; exit 0;;
    -*) echo "Unknown option: $1" >&2; echo "$HELP_MESSAGE"; exit 1;;
  esac
done

BUILD_FLAGS+=(--build-arg config.developer.caching.id="${CACHEID}")
if [[ ! -z "${BASE_VERSION}" ]]; then
  BUILD_FLAGS+=(--build-arg config.build.image="${BASE_VERSION}")
fi
if [[ ! -z "${CL_VERSION}" ]]; then
BUILD_FLAGS+=(--build-arg config.build.dependencies.@aica/foss/control-libraries="${CL_VERSION}")
fi
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" -f aica-package.toml ${*} . || exit 1
