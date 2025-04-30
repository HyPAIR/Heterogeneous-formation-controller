if(NOT lcm_DIR)
  set(lcm_DIR /home/criss/drake/build/install/lib/lcm/cmake)
endif()

find_package(lcm REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/bot2-coreTargets.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/bot2-coreJavaTargets.cmake)
