# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/ros/noetic/include/ompl-1.6;/usr/include;/usr/include/eigen3".split(';') if "${prefix}/include;/opt/ros/noetic/include/ompl-1.6;/usr/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "gazebo_ros;roscpp;roslib;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lheterogeneous_formation_controller;/opt/ros/noetic/lib/x86_64-linux-gnu/libompl.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;-lode".split(';') if "-lheterogeneous_formation_controller;/opt/ros/noetic/lib/x86_64-linux-gnu/libompl.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;-lode" != "" else []
PROJECT_NAME = "heterogeneous_formation_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
