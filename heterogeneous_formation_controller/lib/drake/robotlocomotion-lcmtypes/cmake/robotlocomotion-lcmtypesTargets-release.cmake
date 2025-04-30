#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robotlocomotion-lcmtypes" for configuration "Release"
set_property(TARGET robotlocomotion-lcmtypes APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robotlocomotion-lcmtypes PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "lcm"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "lcm-coretypes;lcmtypes_bot2-core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobotlocomotion-lcmtypes.so"
  IMPORTED_SONAME_RELEASE "librobotlocomotion-lcmtypes.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robotlocomotion-lcmtypes )
list(APPEND _IMPORT_CHECK_FILES_FOR_robotlocomotion-lcmtypes "${_IMPORT_PREFIX}/lib/librobotlocomotion-lcmtypes.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
