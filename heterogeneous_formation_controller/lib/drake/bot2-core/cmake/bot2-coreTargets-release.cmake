#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lcmtypes_bot2-core" for configuration "Release"
set_property(TARGET lcmtypes_bot2-core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lcmtypes_bot2-core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblcmtypes_bot2-core.so"
  IMPORTED_SONAME_RELEASE "liblcmtypes_bot2-core.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lcmtypes_bot2-core )
list(APPEND _IMPORT_CHECK_FILES_FOR_lcmtypes_bot2-core "${_IMPORT_PREFIX}/lib/liblcmtypes_bot2-core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
