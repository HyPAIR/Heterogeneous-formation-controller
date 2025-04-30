/**
 * file yaml_all.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief utilise yaml storing data/reults
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include "optimizer_interface.h"
#include <iostream>
#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace heterogeneous_formation_controller {

bool isFileEmpty(const std::string& filename);
void writePriorityToYAML(const std::vector<std::vector<int>>& priority_set_all, const std::string& filename);
void writeObstacleToYAML(const std::vector<std::vector<math::Vec2d>>& obstacle, const std::string& filename);
void writeTrajectoryToYAML_FG(const std::vector<std::vector<std::vector<double>>>& plan, const int numState, const std::string& filename);
void writeTrajectoryToYAML(const FullStates& trajectory, const std::string& filename);
void writeVectorToYAML(const std::vector<double>& data, const std::string& file_path);
void delete_yaml(const std::string& filename);
std::vector<std::vector<int>> generate_priority_set(std::string filename);
Trajectory_temp fulltotraj(const FullStates& full_traj);
std::vector<std::vector<std::vector<double>>> generate_traj_fg(std::string filename, int numR);
Trajectory_temp generate_traj(std::string filename);
FullStates generate_ref_traj_diff(Trajectory_temp traj_lead, const double offset, const double angle);

}