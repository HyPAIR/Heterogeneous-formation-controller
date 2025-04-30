#!/bin/bash
ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9
rosclean purge 
chmod +x /demo.sh
roslaunch trajectory_tracking heterogeneous_rectangle.launch 