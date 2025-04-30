// Created by weijian on 17/11/23.
#include "traj_tracking/matplotlibcpp.h"
#include "heterogeneous_formation_controller/vehicle_model.h"
#include "traj_tracking/generate_vertex.h"
#include <cmath>
#include <vector>

namespace plt = matplotlibcpp;
using namespace heterogeneous_formation_controller;
using namespace math;

TwoWheelModel twowheel;
GenerateCar car;

void plot_circle (const double& x, const double& y, int index, int pose_index) {
    std::map<std::string, std::string> style;      
    double radius = twowheel.radius;
    std::vector<double> x_, y_;
    switch (index)
    {
    case 1:
        style = {{"color", "blue"}, {"label", "Turtlebot1 Original Trajectory"}};
        break;   
    case 2:
        style = {{"color", "red"}, {"label", "Turtlebot1 Real Trajectory"}};
        break;  
    case 3:
        style = {{"color", "blue"},{"label", "Turtlebot2 Original Trajectory"}};
        break;   
    case 4:
        style = {{"color", "red"}, {"label", "Turtlebot2 Real Trajectory"}};
        break;   
    default:
        break;
    }

    for (double i = 0; i < 36; i++) {
        x_.push_back(x + radius * cos(i * 10 * 2 * M_PI / 360));
        y_.push_back(y + radius * sin(i * 10 * 2 * M_PI / 360));
    }
    plt::plot(x_, y_, style);
    // if (pose_index == 0) {
    //     plt::legend();
    // }
}

void plot_car (const double& x, const double& y, const double& theta, int index, int pose_index) {
    std::map<std::string, std::string> style;      
    double radius = twowheel.radius;
    std::vector<double> x_, y_;
    switch (index)
    {
    case 1:
        style = {{"color", "blue"}, {"label", "Car-like Robot1 Original Trajectory"}};
        break;   
    case 2:
        style = {{"color", "red"}, {"label", "Car-like Robot1 Real Trajectory"}};
        break;  
    case 3:
        style = {{"color", "blue"},{"label", "Car-like Robot2 Original Trajectory"}};
        break;   
    case 4:
        style = {{"color", "red"}, {"label", "Car-like Robot2 Real Trajectory"}};
        break;   
    default:
        break;
    }
    Vec2d center(x, y);
    std::vector<Vec2d> vertex = car.rotatePoint(center, theta);
    vertex.push_back(vertex[0]);
    for (int j = 0; j < vertex.size(); j++) {
        x_.push_back(vertex[j].x());
        y_.push_back(vertex[j].y());
    }
    plt::plot(x_, y_, style);
    // if (pose_index == 0) {
    //     plt::legend();
    // }
}