cmake_minimum_required(VERSION 3.0.2)
project(ewellix_tlt)

add_compile_options(-std=c++11)


set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  serial
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS} )

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES serial ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  DEPENDS boost
)

###########
## Build ##
###########


include_directories(include 
  ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
  src/tlt_node.cpp
  src/serial_com_tlt.cpp
 )


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(
   ${PROJECT_NAME}_node
   serial
   ${catkin_LIBRARIES}
 )
 

#############
## Install ##
#############


#############
## Testing ##
#############

