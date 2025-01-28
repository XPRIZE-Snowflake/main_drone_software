# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_low_alt_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED low_alt_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(low_alt_FOUND FALSE)
  elseif(NOT low_alt_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(low_alt_FOUND FALSE)
  endif()
  return()
endif()
set(_low_alt_CONFIG_INCLUDED TRUE)

# output package information
if(NOT low_alt_FIND_QUIETLY)
  message(STATUS "Found low_alt: 0.0.1 (${low_alt_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'low_alt' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT low_alt_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(low_alt_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${low_alt_DIR}/${_extra}")
endforeach()
