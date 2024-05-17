cmake_minimum_required(VERSION 3.15)
project(modulo_controllers)

# default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

option(ENABLE_SIGNING "Enable signing and its related checks" OFF)
if (ENABLE_SIGNING)
    add_definitions(-DENABLE_SIGNING)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(control_libraries 7.4.0 CONFIG REQUIRED COMPONENTS state_representation robot_model controllers)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/ControllerInterface.cpp
        src/RobotControllerInterface.cpp)

target_link_libraries(${PROJECT_NAME} ${control_libraries_LIBRARIES})

install(DIRECTORY ./controller_descriptions
        DESTINATION .)

ament_auto_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  include_directories(test/include)

  ament_add_gmock(
    test_modulo_controllers
    test/test_controller_interface.cpp
  )
  target_include_directories(test_modulo_controllers PRIVATE include)
  target_link_libraries(test_modulo_controllers ${PROJECT_NAME})
endif()
