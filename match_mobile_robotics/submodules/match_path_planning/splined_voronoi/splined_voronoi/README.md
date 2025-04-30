# Splined Voronoi Planner

This Package contains a global path planner for multi robot formations based on voronoi-diagrams and path smoothing.

## Installation

Need nlopt built from source which is contained as git submodules

```bash
cd path/to/match_path_planning
git submodule update --init --recursive
cd splined_voronoi
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

Install dependencies from rosdep:
```bash
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y
```

## Overview

Implements global path planner splined_voronoi/SplinedVoronoiPlanner which can be used in move base and move base flex.

Performs:
- costmap inflation for multi robot formation
- voronoi generation via boost library
- path planning on voronoi diagram
- selection of waypoints from path
- smoothing of path with quintic Bézier-Splines
- optimization of Splines for collision avoidance and curvature constraint
- sampling of optimized curve and output


### Available Parameters (can be adapted in yaml or via dynamic reconfigure):
- free_cell_threshold: threshold for defining free space from costmap. Should be chosen so that a single robot is guaranteed without collision.
- angle_threshold: minimum angle in orientation between two point for waypoint selection from initial path
- min_distance_control_points: minum distance between two points for waypoint selection
- plan_resolution: sampling resolution of output path (points per meter)
- max_curvature: maximum curvature that robot formation allows (1/formation_radius)
- curvature_safety_margin: percentual margin by which max_curvature is reduced to ensure feasability of resulting path
- free_space_factor: factor for min_radius. At this distance to obstacles freespace is added to planning space regardless of voronoi diagram
- optimize_length: if tangent length of spline should also be optimized; better result if enabled
- max_optimization_time: maximum allowed time for optimization of spline; will cancel optimization afterwards and have non fitting path as output
- formation_config: contains information on relative positions of robots to formation center; will be used to calculate formation radius and allowed max_curvature; planner will take lower value compared to specified curvature for planning
    - robot_names: contains namespace names of all robots; must have leading slash
    - example_name:
        - rel_x_offset: relative offset in x direction
        - rel_y_offset: relative offset in y direction

## How to test

Download and build https://github.com/match-ROS/match_mobile_robotics

```
    git clone https://github.com/match-ROS/match_mobile_robotics
    cd match_mobile_robotics
    ./setup.sh
```
Source your worksspace and run the example launch file.

```
    cd ../..
    source devel/setup.bash
    roslaunch mir_examples formation_path_planning.launch 
```


