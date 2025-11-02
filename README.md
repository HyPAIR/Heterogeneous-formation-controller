<div align ="center">
<h3> RAS 2024: A Decoupled Solution to Heterogeneous Multi-Formation Planning and Coordination for Object Transportation  </h3>

Weijian Zhang, Charlie Street, Masoumeh Mansouri

University of Birmingham

<a href="https://doi.org/10.1016/j.robot.2024.104773"><img alt="Paper" src="https://img.shields.io/badge/Paper-Elsevier-pink"/></a>
<a href="https://youtu.be/A_S-e0mkLGY"><img alt="Video" src="https://img.shields.io/badge/Video-Youtube-red"/></a>
</div>

## Overview
A cooperative formation object transportation system for heterogeneous multi-robot systems that captures robot dynamics and avoids inter-formation collisions.
Our paper has been selected as the **Best Poster Award at The 7th IEEE UK & Ireland RAS Conference**. 
<p align="center">
  <img src="https://github.com/HyPAIR/CPDOT/blob/main/formation_planner/fig/video.gif" alt="A Decoupled Solution to Heterogeneous Multi-Formation Planning and Coordination for Object Transportation" width="600">
</p>
<table style="width:100%; text-align:center;">
  <tr>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo1%20(1).png" alt="Image 1" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo2%20(1).png" alt="Image 2" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo3%20(1).png" alt="Image 3" width="352" height="200"></td>
  </tr>
  <tr>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo4%20(1).png" alt="Image 4" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo5%20(1).png" alt="Image 5" width="352" height="200"></td>
    <td><img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/gazebo6%20(1).png" alt="Image 6" width="352" height="200"></td>
  </tr>
</table>

## Features

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/system_overview.png" alt="system_overview" width="717" height=300">
</p>

 - A comprehensive H-MFPC framework which integrates formation generation, planning, and coordination techniques for heterogeneous formations.
 
 - An efficient formation generation approach for heterogeneous multi-robot systems which synthesizes collision-free and kinematically feasible trajectories in unstructured environments.
   
 - A cost-optimal formation planning method that maintains rigidity for heterogeneous formations.
   
 - A loosely-coupled multi-formation coordination algorithm for ensuring deadlock-free and collision-free navigation among formations.

## Requirements

 - ROS Noetic or later
 - Ubuntu 20.04 or later
 - yaml-cpp 0.8.0 or later
 - You'll also need a license for the Mosek optimization toolbox https://www.mosek.com/ (this package includes a downloader for the Mosek code, but you have to get your own license). Mosek has free licenses available for academic use.

## Installation

1. Create a new workspace:

```shell
$ mkdir -p ~/hmfpc_ws/src
$ cd ~/hmfpc_ws/src
$ catkin_init_workspace
```

2. Clone the package into the workspace:

```shell
$ git clone https://github.com/HyPAIR/Heterogeneous-formation-controller.git
```

3. Install dependencies:
```shell
$ cd ~/hmfpc_ws
$ rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```shell
$ catkin_make
```

## Test in Rviz

1. Launch the simulation to visualize the task allocation result (24 robots with 12 car-like and 12 diff drive in 10 random obstacle environments):

    ```shell
    $ roslaunch heterogeneous_formation_controller demo_ta_test.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/ta_test.png" alt="task_allocation" width="400" height="400">
</p>

2. Launch the simulation to visualize the formation generation result (24 robots with 12 car-like and 12 diff-drive in 10 random obstacle environments):

    ```shell
    $ roslaunch heterogeneous_formation_controller demo_fg_test.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fg_test.png" alt="formation_generation" width="400" height=400">
</p>

3. Launch the simulation to visualize the formation planning result (an irregular formation with 3 car-like and 4 diff-drive in 10 random obstacle environments)):

    ```shell
    $ roslaunch heterogeneous_formation_controller demo_fp_test.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fp_test.png" alt="formation_planning" width="400" height=400">
</p>

4. Launch the simulation to visualize the formation coordination result (2 triangular formations and 3 rectangular formations, wtih 10 car-like and 8 diff-drive in 10 random obstacle environments):

    ```shell
    $ roslaunch heterogeneous_formation_controller fcoord_test.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fc_test.png" alt="formation_coordination" width="400" height=400">
</p>

## Test in Gazebo

1. Launch formation generation simulation, with 10 robots (6 car-like and 4 diff-drive) in 7 random obstacle environments.

    ```shell
   roslaunch heterogeneous_formation_controller formation_generation.launch
    ```
    
    Launch the control node:
    ```shell
   roslaunch heterogeneous_formation_controller fg_test.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fg_real_10.png" alt="fg_10_real" width="800" height=400">
</p>


2. Launch formation planning simulation, with an irregular formation (3 car-like and 4 diff-drive) in a narrow corridor scene.

    ```shell
   roslaunch heterogeneous_formation_controller heterogeneous_triangle.launch
    ```
    
    Launch the control node:
    ```shell
   roslaunch heterogeneous_formation_controller control_triangular_formation.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fp_irregular.png" alt="fp_irregular" width="800" height="400">
</p>

2. Launch formation coordination simulation, with 2 triangular formations (4 car-like and 2 diff-drive) in a "H" environment.

    ```shell
   roslaunch heterogeneous_formation_controller formation_coordination.launch
    ```
    
    Launch the control node:
    ```shell
   roslaunch heterogeneous_formation_controller formation_coordination_demo.launch
    ```

<p align="center">
  <img src="https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/formation_coordination.png" alt="formation_coordination_demo" width="800" height="400">
</p>

## Video

A simulation video demonstrating our proposed framework can be found at [bilibili](https://www.bilibili.com/video/BV1DB421B7bp/?spm_id_from=333.1387.list.card_archive.click&vd_source=bf49c74265570abfae0e3bacc588f839)/[youtube](https://youtu.be/A_S-e0mkLGY).

## Citation

If you find this work useful, please cite [A decoupled solution to heterogeneous multi-formation planning and coordination for object transportation](https://www.sciencedirect.com/science/article/pii/S092188902400157X) ([pdf](http://SimonZhang1999.github.io/files/ras_2024.pdf)):

```bibtex
@article{zhang2024decoupled,
  title={A decoupled solution to heterogeneous multi-formation planning and coordination for object transportation},
  author={Zhang, Weijian and Street, Charlie and Mansouri, Masoumeh},
  journal={Robotics and Autonomous Systems},
  pages={104773},
  year={2024},
  publisher={Elsevier}
}
```
