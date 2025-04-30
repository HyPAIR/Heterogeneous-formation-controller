#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ccd" for configuration "Release"
set_property(TARGET ccd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ccd PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib/x86_64-linux-gnu/libm.so"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libccd.so.2.0"
  IMPORTED_SONAME_RELEASE "libccd.so.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS ccd )
list(APPEND _IMPORT_CHECK_FILES_FOR_ccd "${_IMPORT_PREFIX}/lib/libccd.so.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
