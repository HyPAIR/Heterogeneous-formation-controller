#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "yaml-cpp" for configuration "Release"
set_property(TARGET yaml-cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(yaml-cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/criss/drake/build/install/lib/libyaml-cpp.so.0.5.3"
  IMPORTED_SONAME_RELEASE "libyaml-cpp.so.0.5"
  )

list(APPEND _IMPORT_CHECK_TARGETS yaml-cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_yaml-cpp "/home/criss/drake/build/install/lib/libyaml-cpp.so.0.5.3" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
