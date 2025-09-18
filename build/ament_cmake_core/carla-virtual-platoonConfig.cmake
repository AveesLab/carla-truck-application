# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_carla-virtual-platoon_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED carla-virtual-platoon_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(carla-virtual-platoon_FOUND FALSE)
  elseif(NOT carla-virtual-platoon_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(carla-virtual-platoon_FOUND FALSE)
  endif()
  return()
endif()
set(_carla-virtual-platoon_CONFIG_INCLUDED TRUE)

# output package information
if(NOT carla-virtual-platoon_FIND_QUIETLY)
  message(STATUS "Found carla-virtual-platoon: 0.0.0 (${carla-virtual-platoon_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'carla-virtual-platoon' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${carla-virtual-platoon_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(carla-virtual-platoon_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${carla-virtual-platoon_DIR}/${_extra}")
endforeach()
