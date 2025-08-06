# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pinky_rmf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pinky_rmf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pinky_rmf_FOUND FALSE)
  elseif(NOT pinky_rmf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pinky_rmf_FOUND FALSE)
  endif()
  return()
endif()
set(_pinky_rmf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pinky_rmf_FIND_QUIETLY)
  message(STATUS "Found pinky_rmf: 0.0.0 (${pinky_rmf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pinky_rmf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT pinky_rmf_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pinky_rmf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pinky_rmf_DIR}/${_extra}")
endforeach()
