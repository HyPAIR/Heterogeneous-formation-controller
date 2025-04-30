cmake_minimum_required(VERSION 2.8.3)
project(splined_voronoi)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cv_bridge
  rospy
  std_msgs
  sensor_msgs
  costmap_2d
  dynamic_reconfigure
)

# # System dependencies are found with CMake's conventions
find_package(OpenCV)

find_package(Eigen3 REQUIRED)

# set(tinysplinecxx_DIR "/usr/local/lib64/cmake/tinysplinecxx/")
# find_package(tinysplinecxx)

# set(nlopt_DIR "/usr/local/lib/cmake/nlopt/")
find_package(NLopt)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


add_service_files(
  FILES
  MakePlanWithStats.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  config/SplinedVoronoiPlanner.cfg
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  INCLUDE_DIRS include
  LIBRARIES splined_voronoi
  # CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
  # DEPENDS system_lib
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${TINYSPLINECXX_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)


add_library(splined_voronoi src/splined_voronoi_planner.cpp src/path_planning src/path_smoothing src/voronoi_generation)
target_link_libraries(splined_voronoi
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # ${TINYSPLINECXX_LIBRARIES}
  ${NLOPT_LIBRARIES}
)
add_dependencies(splined_voronoi splined_voronoi_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
