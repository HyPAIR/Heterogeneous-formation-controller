# match Mobile Robotics

## 📝 Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
- [External Documentation](#docs)
- [Problem Solving](#problems)
- [TODO](../TODO.md)

## 1. Repository overview <a name = "overview"></a>
* `general_hardware_helper`: Contains scripts and packages that help when using the hardware robots
* `general_sim_helper`: Contains scripts and packages that help when using the simulation robots
* `igus`: Contains all scripts and packages for the usage of the Igus robot
* `match_gazebo`: Contains all worlds withs maps and launch files for the Gazebo simulation
* `mir`: Contains all information (description, examples and launch files) for the MiR platform
* `miranda`: Contains all information (description, examples and launch files) for the MiR platform with the Panda robot
* `mur`: Contains all information (description, examples and launch files) for the MiR platform with the UR5 robot
* `net_box_hardware_helper`: Contains all scripts and launch files for the driver of the NetBox hardware
* `panda`: Contains the original Panda repos and additional controllers, scripts and launch files
* `scout_mini`: Contains source and launch files for the Scout Mini plattform
* `ur`: Contains the original UR repos and additional controllers, scripts and launch files

## 2. Installation <a name = "installation"></a>
Start by changeing directory to your catkin workspace!
### Clone package
```
git clone https://github.com/ROSmatch/match_mobile_robotics.git
```
### Install dependencies
Browse into `your_catkin_ws_name/src/Match_Mobile_Robotics` and execute
```
git submodule update --init --recursive
cd ../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
If the last `rosdep install` command is not working, please try the following:
```
rosdep install --from-paths src --ignore-src -r -y
```

### Build packages
Use your standard build tools to build the downloaded packages e.g. : 
```
catkin build
```

## 3. Usage <a name="usage"></a>
For examples on how to use this repository stick to the "mir_examples" package.

```
roslaunch mir_examples single_mir_100.launch
```

## 4. External doc <a name="docs"></a>:
[Ur dashboard](https://s3-eu-west-1.amazonaws.com/ur-support-site/15690/Dashboard_Server_CB-Series.pdf)  
[UR Ros driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/64ab483b550a1c079c70162d2d3c2eb21ecde76e)  
[UR Robot](https://github.com/fmauch/universal_robot/tree/3ebf8070ad0869c264fc3df9185fe1865773b2b4)  
[Franka (Panda)](https://frankaemika.github.io/docs/index.html)  

## 5. Problem Solving 🔧  <a name="problems"></a>

### AttributeError: module 'distutils' has no attribute 'core'
Arises at Build. Solved by installing other version of setup_tools:

```bash
  pip install setuptools==59.5.0
```
