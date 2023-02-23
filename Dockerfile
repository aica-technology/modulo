ARG BASE_TAG=humble
FROM ghcr.io/aica-technology/ros2-control-libraries:${BASE_TAG} as dependencies
WORKDIR ${HOME}/ros2_ws


FROM dependencies as modulo-component-interfaces

COPY --chown=${USER} ./source/modulo_component_interfaces ./src/modulo_component_interfaces
COPY --chown=${USER} ./source/modulo_utils ./src/modulo_utils
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"


FROM modulo-component-interfaces as modulo-core

COPY --chown=${USER} ./source/modulo_core ./src/modulo_core
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --packages-select modulo_core"


FROM modulo-core as modulo-components

COPY --chown=${USER} ./source/modulo_components ./src/modulo_components
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --packages-select modulo_components"

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
