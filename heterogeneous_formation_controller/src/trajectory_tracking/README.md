# trajectory_tracking

[![Project Image](https://img.youtube.com/vi/8TWlnW2JLhw/0.jpg)](https://www.youtube.com/watch?v=8TWlnW2JLhw)

> ROS based trajectory_tracking GAZEBO simulation.

---

### Table of Contents

- [Description](#description)
- [Installation](#Installation)
- [How To Use](#how-to-use)
- [Author Info](#author-info)

---

## Description

A simple ROS package of a trajectory_tracking GAZEBO simulation using turtlebot3 robot model based on three diffrent controllers (PID, Backstepping, Feedbak linearization, model predictive) .

[Back To The Top](#trajectory_tracking)

---

## Installation

1- Go to your catkin_ws directory
```shell
    cd ~/catkin_ws/src
```
2- Clone the repository
```shell
    git clone https://github.com/alla-eddine/trajectory_tracking.git
```
3- install by 
```shell
    cd ..
    catkin_make
```
---

## How To Use

1- Go to your catkin_ws directory
```shell
    cd ~/catkin_ws
    catkin_make
```
2- in the first terminal tap run
```shell
    roslaunch trajectory_tracking gazebo_tb3_sim.launch
```
3- in the second terminal tap run

A- for backstepping controller run 
```shell
    roslaunch trajectory_tracking backstepping.launch
```
B- for feedback linearization controller run 
```shell
    roslaunch trajectory_tracking feedback.launch
```
C- for pid controller run 
```shell
    roslaunch trajectory_tracking pid.launch
```
D- for model predictive controller run 
```shell
    roslaunch trajectory_tracking mpc.launch
```

[Back To The Top](#trajectory_tracking)

---



## Author Info

- Twitter - [@D_allaeddine](https://twitter.com/D_allaeddine)
- Website - [ROS in Arabic](arabic-ros.eb2a.com)

[Back To The Top](#trajectory_tracking)
