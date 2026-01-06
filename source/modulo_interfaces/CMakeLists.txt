cmake_minimum_required(VERSION 3.15)
project(modulo_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB SRVS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
rosidl_generate_interfaces(${PROJECT_NAME} ${MSGS} ${SRVS} DEPENDENCIES rcl_interfaces std_msgs)

ament_package()