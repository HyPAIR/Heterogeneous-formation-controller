git submodule update --init --recursive
cd splined_voronoi/nlopt/
cmake .
make
sudo make install

cd ../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash
