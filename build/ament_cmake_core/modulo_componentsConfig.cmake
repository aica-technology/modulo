# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_modulo_components_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED modulo_components_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(modulo_components_FOUND FALSE)
  elseif(NOT modulo_components_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(modulo_components_FOUND FALSE)
  endif()
  return()
endif()
set(_modulo_components_CONFIG_INCLUDED TRUE)

# output package information
if(NOT modulo_components_FIND_QUIETLY)
  message(STATUS "Found modulo_components: 3.2.0 (${modulo_components_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'modulo_components' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT modulo_components_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(modulo_components_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${modulo_components_DIR}/${_extra}")
endforeach()
