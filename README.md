# Heterogeneous Multi-formation Planning and Coordination Framework

A cooperative formation object transportation system for heterogeneous multi-robot systems that captures robot dynamics and avoids inter-formation collisions. 

## Features

 - A comprehensive H-MFPC framework which integrates formation generation, planning, and coordination techniques for heterogeneous formations.
 - An efficient formation generation approach for heterogeneous multi-robot systems which synthesizes collision-free and kinematically feasible trajectories in unstructured environments.
 - A cost-optimal formation planning method that maintains rigidity for heterogeneous formations.
 - A loosely-coupled multi-formation coordination algorithm for ensuring deadlock-free and collision-free navigation among formations.

## Requirements

 - ROS Noetic or later
 - Ubuntu 20.04 or later

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
rosdep install heterogeneous_formation_controller
```

3. Build the workspace:

```shell
$ cd ~/hmfpc_ws
$ catkin_make
```

## Test in Rviz

1. Launch the simulation to visualize the task allocation result (24 robots with 12 car-like and 12 diff drive in 22 random obstacle environments):

    ```shell
    $ roslaunch Heterogeneous-formation-controller demo_ta_test.launch
    ```


![task_allocation](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/ta_test.png)

2. Launch the simulation to visualize the formation generation result (24 robots with 12 car-like and 12 diff-drive in 22 random obstacle environments):

    ```shell
    $ roslaunch Heterogeneous-formation-controller demo_fg_test.launch
    ```

![formation_generation](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fg_test.png)

3. Launch the simulation to visualize the formation planning result (a triangular formation with 2 car-like and 1 diff-drive in a narrow corridor scene)):

    ```shell
    $ roslaunch Heterogeneous-formation-controller demo_fp_test.launch
    ```

![formation_planning](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fp_test.png)

4. Launch the simulation to visualize the formation coordination result (2 triangular formations wtih 4 car-like and 2 diff-drive in a "H" environment):

    ```shell
    $ roslaunch Heterogeneous-formation-controller fcoord_test.launch
    ```

![formation_coordination](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fc_test.png)

## Test in Gazebo

1. Launch formation generation simulation, with 10 robots (6 car-like and 4 diff-drive) in 7 random obstacle environments.

    ```shell
   roslaunch Heterogeneous-formation-controller formation_generation.launch
    ```
    
    Launch the control node:
    ```shell
   roslaunch Heterogeneous-formation-controller fg_test.launch
    ```
![fg_10_real](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fg_real_10.png)

2. Launch formation planning simulation, with a triangular formation (2 car-like and 1 diff-drive) in a narrow corridor scene.

    ```shell
   roslaunch Heterogeneous-formation-controller heterogeneous_triangle.launch
    ```
    
    Launch the control node:
    ```shell
   roslaunch Heterogeneous-formation-controller control_triangular_formation.launch
    ```
![fp_tri_corridor](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/fp_triangle.png)

2. Launch formation coordination simulation, with 2 triangular formations (4 car-like and 2 diff-drive) in a "H" environment.

    ```shell
   roslaunch Heterogeneous-formation-controller formation_coordination.launch
    ```
    
    Launch the control node:
    ```shell
   roslaunch Heterogeneous-formation-controller formation_coordination_demo.launch
    ```
![formation_coordination_demo](https://github.com/HyPAIR/Heterogeneous-formation-controller/blob/main/Figures/formation_coordination.png)

## Video

A simulation video demonstrating our proposed framework can be found at [youtube](https://youtu.be/LLzFdUBqXVw).