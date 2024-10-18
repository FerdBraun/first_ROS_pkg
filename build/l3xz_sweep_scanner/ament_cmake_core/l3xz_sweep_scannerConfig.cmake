# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_l3xz_sweep_scanner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED l3xz_sweep_scanner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(l3xz_sweep_scanner_FOUND FALSE)
  elseif(NOT l3xz_sweep_scanner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(l3xz_sweep_scanner_FOUND FALSE)
  endif()
  return()
endif()
set(_l3xz_sweep_scanner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT l3xz_sweep_scanner_FIND_QUIETLY)
  message(STATUS "Found l3xz_sweep_scanner: 1.0.0 (${l3xz_sweep_scanner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'l3xz_sweep_scanner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${l3xz_sweep_scanner_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(l3xz_sweep_scanner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${l3xz_sweep_scanner_DIR}/${_extra}")
endforeach()
