cmake_minimum_required(VERSION 3.0.2)
project(heterogeneous_formation_controller)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DHAVE_CSTDDEF -D_USE_MATH_DEFINES)

#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)
#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
#EIGEN
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#OCTOMAP
find_package(octomap_ros REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#Decomp
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  base_local_planner
  costmap_2d
  gazebo_ros
  geometry_msgs
  nav_core
  nav_msgs
  std_msgs
  pluginlib
  tf2
  tf2_ros
  visualization_msgs
  interactive_markers
  octomap_ros
  octomap_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(ompl REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES heterogeneous_formation_controller
  CATKIN_DEPENDS gazebo_ros roscpp roslib message_runtime
  DEPENDS OMPL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/iris
  ${PROJECT_SOURCE_DIR}/include/iris/iris
  ${PROJECT_SOURCE_DIR}/include/iris/eigen3
  "/usr/include/python3.8/"
)

find_library(IRIS_LIB iris HINTS ${PROJECT_SOURCE_DIR}/lib/iris)
find_library(IRIS_GEOMETRY_LIB iris_geometry HINTS ${PROJECT_SOURCE_DIR}/lib/iris)
find_library(IRIS_CVXGEN_LDP_CPP_LIB NAMES iris_cvxgen_ldp_cpp HINTS ${PROJECT_SOURCE_DIR}/lib/iris)
find_library(IRIS_CVXGEN_LDP_LIB iris_cvxgen_ldp HINTS ${PROJECT_SOURCE_DIR}/lib/iris)
# Change the path to your mosek license
find_library(MOSEK_LIB NAMES mosek64 HINTS "${MOSEK_DIR}/tools/platform/linux64x86/bin")

# Declare a C++ library
add_library(heterogeneous_formation_controller
  src/formation_planning/heterogeneous_formation_controller_ros.cpp
  src/formation_planning/lightweight_nlp_problem.cpp
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/visualization/color.cpp
  src/visualization/plot.cpp
  src/formation_planning/heterogeneous_formation_controller.cpp
  src/formation_planning/coarse_path_planner.cpp
  src/formation_planning/environment.cpp
  src/yaml.cpp
  # src/formation_planning/topo_prm.cpp
  # src/formation_planning/IdentifyHomotopy.cpp
  # src/formation_planning/forward_kinematics.cpp
)
target_include_directories(heterogeneous_formation_controller PUBLIC ${PROJECT_SOURCE_DIR}lib/iris)
target_include_directories(heterogeneous_formation_controller PUBLIC "${MOSEK_DIR}/tools/platform/linux64x86/bin")
target_link_libraries(heterogeneous_formation_controller ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ipopt adolc ${IRIS_LIB} ${IRIS_GEOMETRY_LIB} ${MOSEK_LIB} ${IRIS_CVXGEN_LDP_CPP_LIB})
target_link_libraries(heterogeneous_formation_controller /usr/local/lib/libyaml-cpp.so.0.8)

add_executable(hmfpc_test_node src/formation_planning/hmfpc_test_node.cpp)
target_link_libraries(hmfpc_test_node ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(hmfpc_test_node /usr/local/lib/libyaml-cpp.so.0.8)

# add_executable(topologic_pp src/formation_planning/topologic_pp.cpp)
# target_link_libraries(topologic_pp ${catkin_LIBRARIES} heterogeneous_formation_controller)
# target_link_libraries(topologic_pp /usr/local/lib/libyaml-cpp.so.0.8)

#BUILD
add_executable(random_map_generator src/formation_generation/random_map_generator.cpp)
target_link_libraries(random_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(warehouse_map_generator src/formation_generation/warehouse_map_generator.cpp)
target_link_libraries(warehouse_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# add_executable(coupled_traj_planner
#   src/formation_generation/coupled_traj_planner.cpp
# )
# target_link_libraries(coupled_traj_planner
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_INCLUDE_DIRS}
#   ${SIPP_LINK_LIBS}
#   ${PYTHON_LIBRARIES}
#   m
#   ipopt
# )


add_executable(prioritized_traj_planner
  src/formation_generation/prioritized_traj_planner.cpp
)
target_link_libraries(prioritized_traj_planner
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
  heterogeneous_formation_controller
  /usr/local/lib/libyaml-cpp.so.0.8
)

add_executable(ta_test_node src/task_assignment/ta_test_node.cpp src/task_assignment/Hungarian.cpp)
target_link_libraries(ta_test_node ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(ta_test_node /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(ta_test_node
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
)

add_executable(fg_test_node src/formation_generation/fg_test_node.cpp)
target_link_libraries(fg_test_node ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(fg_test_node /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(fg_test_node
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(animation src/formation_generation/animation.cpp)
target_link_libraries(animation ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(animation /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(animation
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(test_decomp src/decomp_util/test_decomp.cpp)
target_link_libraries(test_decomp ${Boost_LIBRARIES})
target_link_libraries(test_decomp ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(test_decomp /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(test_decomp
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)
add_test(test_seed_decomp test_seed_decomp)

add_executable(generate_decomp src/decomp_util/generate_decomp.cpp)
target_link_libraries(generate_decomp ${Boost_LIBRARIES})
target_link_libraries(generate_decomp ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(generate_decomp /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(generate_decomp
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(control_triangular_formation src/trajectory_tracking/fc_tracking_controller/control_triangular_formation.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(control_triangular_formation ${catkin_LIBRARIES})
target_link_libraries(control_triangular_formation /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(control_triangular_formation PUBLIC /usr/include/eigen3)
target_link_libraries(control_triangular_formation /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(plot_result src/trajectory_tracking/plot_result.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(plot_result ${catkin_LIBRARIES})
target_link_libraries(plot_result /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(plot_result PUBLIC /usr/include/eigen3)
target_link_libraries(plot_result /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(plot_result3 src/trajectory_tracking/plot_result3.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(plot_result3 ${catkin_LIBRARIES})
target_link_libraries(plot_result3 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(plot_result3 PUBLIC /usr/include/eigen3)
target_link_libraries(plot_result3 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(control_rectangle_formation src/trajectory_tracking/fc_tracking_controller/control_rectangle_formation.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(control_rectangle_formation ${catkin_LIBRARIES})
target_link_libraries(control_rectangle_formation /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(control_rectangle_formation PUBLIC /usr/include/eigen3)
target_link_libraries(control_rectangle_formation /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(fg_test_r1 src/trajectory_tracking/fg_tracking_controller/fg_test_r1.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r1 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r1 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r1 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r1 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r1 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r2 src/trajectory_tracking/fg_tracking_controller/fg_test_r2.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r2 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r2 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r2 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r2 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r2 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r3 src/trajectory_tracking/fg_tracking_controller/fg_test_r3.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r3 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r3 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r3 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r3 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r3 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r4 src/trajectory_tracking/fg_tracking_controller/fg_test_r4.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r4 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r4 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r4 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r4 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r4 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r5 src/trajectory_tracking/fg_tracking_controller/fg_test_r5.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r5 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r5 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r5 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r5 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r5 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r6 src/trajectory_tracking/fg_tracking_controller/fg_test_r6.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r6 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r6 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r6 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r6 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r6 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r7 src/trajectory_tracking/fg_tracking_controller/fg_test_r7.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r7 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r7 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r7 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r7 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r7 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r8 src/trajectory_tracking/fg_tracking_controller/fg_test_r8.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r8 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r8 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r8 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r8 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r8 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r9 src/trajectory_tracking/fg_tracking_controller/fg_test_r9.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r9 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r9 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r9 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r9 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r9 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fg_test_r10 src/trajectory_tracking/fg_tracking_controller/fg_test_r10.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(fg_test_r10 ${catkin_LIBRARIES})
target_link_libraries(fg_test_r10 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(fg_test_r10 PUBLIC /usr/include/eigen3)
target_link_libraries(fg_test_r10 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(fg_test_r10 ${catkin_LIBRARIES} heterogeneous_formation_controller)

add_executable(fcoord_test_node src/formation_coordination/fcoord_test.cpp)
target_link_libraries(fcoord_test_node ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(fcoord_test_node /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(fcoord_test_node
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(animation_coord src/formation_coordination/animation.cpp)
target_link_libraries(animation_coord ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(animation_coord /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(animation_coord
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(animation_complete src/animation/complete_animation.cpp)
target_link_libraries(animation_complete ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(animation_complete /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(animation_complete
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(load_data src/sarah_simulation/load_data.cpp)
target_link_libraries(load_data ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(load_data /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(load_data
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(traj_tracking_sim src/sarah_simulation/traj_tracking.cpp)
target_link_libraries(traj_tracking_sim ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(traj_tracking_sim /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(traj_tracking_sim
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(animation_demo_sim src/animation/animation_demo.cpp)
target_link_libraries(animation_demo_sim ${catkin_LIBRARIES} heterogeneous_formation_controller)
target_link_libraries(animation_demo_sim /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(animation_demo_sim
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)
# add_executable(animation_flexible src/formation_planning/animation.cpp src/trajectory_tracking/plot.cpp)
# target_link_libraries(animation_flexible ${catkin_LIBRARIES} heterogeneous_formation_controller)
# target_link_libraries(animation_flexible /usr/local/lib/libyaml-cpp.so.0.8)
# target_include_directories(animation_flexible PUBLIC /usr/include/eigen3)
# target_link_libraries(animation_flexible
#   ${OCTOMAP_INCLUDE_DIRS}
#   ${SIPP_LINK_LIBS}
#   ${PYTHON_LIBRARIES}
#   m
#   ipopt
#   /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so
# )

# add_executable(fk_test src/formation_planning/fk_test.cpp src/trajectory_tracking/plot.cpp)
# target_link_libraries(fk_test ${catkin_LIBRARIES} heterogeneous_formation_controller)
# target_link_libraries(fk_test /usr/local/lib/libyaml-cpp.so.0.8)
# target_include_directories(fk_test PUBLIC /usr/include/eigen3)
# target_link_libraries(fk_test
#   ${OCTOMAP_INCLUDE_DIRS}
#   ${SIPP_LINK_LIBS}
#   ${PYTHON_LIBRARIES}
#   m
#   ipopt
#   /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so
# )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_heterogeneous_formation_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
