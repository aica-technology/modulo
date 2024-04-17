#syntax=docker/dockerfile:1.4.0
ARG CL_VERSION=v7.4.0
ARG ROS2_VERSION=iron
FROM ghcr.io/aica-technology/control-libraries:${CL_VERSION} as cl
FROM ghcr.io/aica-technology/ros2-ws:${ROS2_VERSION} as base
# setup the environment
USER ${USER}
ENV WORKSPACE ${HOME}/ws
WORKDIR ${WORKSPACE}
SHELL ["/bin/bash", "-l", "-c"]

# create a workspace
RUN source ${HOME}/ros2_ws/install/setup.bash && colcon build
# source the new workspace on login
RUN echo "source ${WORKSPACE}/install/setup.bash" | cat - ${HOME}/.bashrc > tmp && mv tmp ${HOME}/.bashrc
# install deps
COPY --from=cl / /
# install sources
COPY --chown=${USER}:${USER} ./source ${WORKSPACE}/src

FROM base as utils-development
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=./build,id=${TARGETPLATFORM}-${CACHEID},uid=1000 \
  colcon build --packages-select modulo_component_interfaces

FROM utils-development as core-development
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=./build,id=${TARGETPLATFORM}-${CACHEID},uid=1000 \
  colcon build --packages-select modulo_utils

FROM core-development as development
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=./build,id=${TARGETPLATFORM}-${CACHEID},uid=1000 \
  colcon build --packages-select modulo_core

FROM base as build
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=./build,id=${TARGETPLATFORM}-${CACHEID},uid=1000 \
  sudo apt-get update && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ros2_control ros2_controllers controller_interface hardware_interface controller_manager" \
  && sudo rm -rf /var/lib/apt/lists/* \
  && colcon build

FROM build as test
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=./build,id=${TARGETPLATFORM}-${CACHEID},uid=1000 \
  colcon test && colcon test-result --verbose

FROM scratch as production
COPY --from=build /home/ros2/ws/install /colcon
