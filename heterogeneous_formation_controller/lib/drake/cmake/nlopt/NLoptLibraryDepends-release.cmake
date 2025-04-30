#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nlopt" for configuration "Release"
set_property(TARGET nlopt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nlopt PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libnlopt.so.0.9.0"
  IMPORTED_SONAME_RELEASE "libnlopt.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS nlopt )
list(APPEND _IMPORT_CHECK_FILES_FOR_nlopt "${_IMPORT_PREFIX}/lib/libnlopt.so.0.9.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
