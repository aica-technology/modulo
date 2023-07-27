#!/bin/bash

STEP=1
build_and_test() {
  PACKAGE=$1

  echo ">>> ${STEP}: Building ${PACKAGE}..."
  cp -r /github/workspace/source/"${PACKAGE}" ./src/"${PACKAGE}"
  if ! colcon build --packages-select "${PACKAGE}"; then
    echo ">>> [ERROR] Build stage ${STEP} failed!"
    exit "${STEP}"
  fi
  echo ">>> Build stage ${STEP} completed successfully!"
  STEP=$((STEP+1))

  echo ">>> ${STEP}: Testing ${PACKAGE}..."
  if ! colcon test --packages-select "${PACKAGE}" --return-code-on-test-failure; then
    colcon test-result --verbose
    echo ">>> [ERROR] Test stage ${STEP} failed!"
    exit "${STEP}"
  fi
  echo ">>> Test stage ${STEP} completed successfully!"
  STEP=$((STEP+1))
}

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd /home/ros2/ros2_ws

build_and_test modulo_component_interfaces
build_and_test modulo_utils
build_and_test modulo_core
build_and_test modulo_components

echo ">>> All build and test stages completed successfully!"
exit 0
