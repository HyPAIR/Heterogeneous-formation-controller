# Mir Hardware Helper
## 0. Package origin
This package came originally from (mir_robot)[https://github.com/dfki-ric/mir_robot].

## 1. Package overview
* `mir_description`: URDF description of the MiR robot
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_launch_sim`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files
* `mir_launch_hardware`  : Standalone launchfiles for some different tasks

## 2. Installation
### Install dependencies
```
git submodule init
git submodule update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
### Build packages
Use your standard build system to build the downloaded packages