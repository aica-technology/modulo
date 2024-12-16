# CMake generated Testfile for 
# Source directory: /home/ros2/ws/src/modulo_components
# Build directory: /home/ros2/.devcontainer/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_modulo_components "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_modulo_components.gtest.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_gtest/test_modulo_components.txt" "--command" "/home/ros2/.devcontainer/build/test_modulo_components" "--gtest_output=xml:/home/ros2/.devcontainer/build/test_results/modulo_components/test_modulo_components.gtest.xml")
set_tests_properties(test_modulo_components PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ros2/.devcontainer/build/test_modulo_components" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/iron/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;52;ament_add_gtest;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_component "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_component.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_component.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_component.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_component PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_component_communication "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_communication.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_communication.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_component_communication.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_communication/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_communication.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_component_communication PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_component_interface "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_interface.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_interface.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_component_interface.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_interface/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_interface.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_component_interface PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_component_interface_empty_parameters "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_interface_empty_parameters.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_interface_empty_parameters.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_component_interface_empty_parameters.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_interface_empty_parameters/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_interface_empty_parameters.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_component_interface_empty_parameters PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_component_interface_parameters "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_interface_parameters.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_interface_parameters.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_component_interface_parameters.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_component_interface_parameters/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_component_interface_parameters.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_component_interface_parameters PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_lifecycle_component "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_lifecycle_component.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_lifecycle_component.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_lifecycle_component.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_lifecycle_component/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_lifecycle_component.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_lifecycle_component PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
add_test(test_lifecycle_component_communication "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/ros2/.devcontainer/build/test_results/modulo_components/test_lifecycle_component_communication.xunit.xml" "--package-name" "modulo_components" "--output-file" "/home/ros2/.devcontainer/build/ament_cmake_pytest/test_lifecycle_component_communication.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/ros2/ws/src/modulo_components/test/python/test_lifecycle_component_communication.py" "-o" "cache_dir=/home/ros2/.devcontainer/build/ament_cmake_pytest/test_lifecycle_component_communication/.cache" "--junit-xml=/home/ros2/.devcontainer/build/test_results/modulo_components/test_lifecycle_component_communication.xunit.xml" "--junit-prefix=modulo_components")
set_tests_properties(test_lifecycle_component_communication PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/ros2/.devcontainer/build" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;64;ament_add_pytest_test;/home/ros2/ws/src/modulo_components/CMakeLists.txt;0;")
subdirs("gtest")