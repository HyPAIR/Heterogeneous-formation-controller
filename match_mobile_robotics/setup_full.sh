# copy meshes to the local gazebo model folder
mkdir ~/.gazebo/models/meshes -p
cp match_gazebo/models/meshes/* /home/$USER/.gazebo/models/meshes

git submodule update --init --recursive
cd submodules/match_path_planning/splined_voronoi/nlopt/
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../../../../../..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash
