/**
 * file prioritized_traj_optimization.hpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief prioritized trajectory optimization using optimal control
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#pragma once

#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <map>
#include <random>
#include <cmath>
#include <unordered_set>
#include <stdlib.h>

// ROS
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>


// Submodules
#include <formation_generation/corridor.hpp>
#include <formation_generation/init_traj_planner.hpp>
#include <formation_generation/mission.hpp>
#include <formation_generation/param.hpp>

#include "com_fun.h"
#include "optimization.h"
#include "time.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

#include "heterogeneous_formation_controller/optimizer_interface.h"

using CppAD::AD;
typedef CppAD::AD<double> ADdouble;

int qi;

double cost;

int diff_count;

int goal_theta_ind = 0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start;
size_t y_start;
size_t theta_start;
size_t v_start;
size_t omega_start;
size_t vertice_start;

std::vector<ros::Publisher> way_point_pub;

std::vector<std::vector<std::array<int, 2>>> relative_pair; // 存储每个时刻相关的pair
std::vector<std::vector<std::array<int, 2>>> relative_pair_updated; // 存储更新后的每个时刻相关的pair
std::vector<std::array<int, 3>> selected_pair_new; // 保存特定同一组(相同优先级)中机器人的碰撞约束
std::vector<std::array<int, 3>> selected_pair_old; // 保存特定一组机器人中与高优先级碰撞约束
std::vector<std::vector<int>> group;
std::vector<std::unordered_set<int>> robot_constraints;
std::vector<double> traj_length_set;
bool initial_traj_get = false;
int group_num;
int group_qn;
std::vector<double> robot_type;
// std::set<int> old_set;

struct Point {
    ADdouble x;
    ADdouble y;
};

struct Rectangle {
    ADdouble centerX;
    ADdouble centerY;
    ADdouble width;
    ADdouble height;
    ADdouble angle; // 方位角（偏转角度）
};

// 计算矩形的四个顶点坐标
void calculateRectangleVertices(const Rectangle& rect, Point vertices[4]) {
    // 将角度转换为弧度
    ADdouble angleRad = rect.angle * M_PI / 180.0;

    // 计算矩形的半宽和半高
    ADdouble halfWidth = rect.width / 2.0;
    ADdouble halfHeight = rect.height / 2.0;

    // 计算矩形的旋转矩阵
    ADdouble cosA = cos(angleRad);
    ADdouble sinA = sin(angleRad);

    // 计算矩形的四个顶点坐标
    vertices[0].x = rect.centerX + halfWidth * cosA + halfHeight * sinA; // 右上角 x
    vertices[0].y = rect.centerY - halfWidth * sinA + halfHeight * cosA; // 右上角 y

    vertices[1].x = rect.centerX - halfWidth * cosA + halfHeight * sinA; // 左上角 x
    vertices[1].y = rect.centerY + halfWidth * sinA + halfHeight * cosA; // 左上角 y

    vertices[2].x = rect.centerX - halfWidth * cosA - halfHeight * sinA; // 左下角 x
    vertices[2].y = rect.centerY + halfWidth * sinA - halfHeight * cosA; // 左下角 y

    vertices[3].x = rect.centerX + halfWidth * cosA - halfHeight * sinA; // 右下角 x
    vertices[3].y = rect.centerY - halfWidth * sinA - halfHeight * cosA; // 右下角 y
}

ADdouble calculateTriangleArea(const Point& A, const Point& B, const Point& C) {
    return 0.5 * CppAD::abs(A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y));
}

struct RobotCost {
    int index;
    double cost;

    // 构造函数
    RobotCost(int ind, double cost) : index(ind), cost(cost) {}
};  
// 定义比较运算符，用于在 multimap 中自动排序
bool compareCost(const RobotCost& a, const RobotCost& b) {
    return a.cost > b.cost;
}

std::vector<RobotCost> costMap;
std::vector<double> robot_cost_set;

class FG_eval {
     public:

      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;


      void operator()(ADvector& fg, const ADvector& vars) {
        // MPC implementation
        // fg a vector of constraints, x is a vector of constraints.
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
      fg[0] = 0; 
      // diff_count = 0;
      // goal_theta_ind = 0;
      // for (int j = 0; j < group_qn; j++) {
      //   qi = group[group_num][j];
      //   if (robot_type[qi] < 1) {
      //     diff_count++;
      //   }
      // }
      for (int j = 0; j < group_qn; j++){
        
        int nu = group[group_num][j];
        ROS_INFO_STREAM("Try to solve Robot_" << nu);
        // i为智能体的数量，N为智能体的waypoint个数，第三个为状态数
        // 惩罚x, y方向上的初始偏离值
        for (int i = 1; i < N - 1; i++) {
          fg[0] += 0.01*W_X * CppAD::pow(vars[x_start + j*N + i] - plan[nu][i][0], 2);
          fg[0] += 0.01*W_Y * CppAD::pow(vars[y_start + j*N + i] - plan[nu][i][1], 2); 
        }
        // 惩罚v, w的初始值
        fg[0] += W_DV * CppAD::pow(vars[v_start+j*(N-1)], 2);
        fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start+j*(N-1)], 2);

        // Minimize the value gap between sequential actuations.
        // 使得控制圆滑：i+1时刻与i时刻的控制指令的差值
        for (int i = 0; i < N - 2; i++) {
          fg[0] += W_DV * CppAD::pow(vars[v_start + j*(N-1) + i + 1] - vars[v_start + j*(N-1)+ i], 2);
          fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start + j*(N-1) + i + 1] - vars[omega_start + j*(N-1) + i], 2);
        }
        // for (int i = N - 30; i < N - 1; i++) {
        //   fg[0] += W_FINAL * (CppAD::pow(vars[x_start + j*N + i + 1] - vars[x_start + j*N + i], 2) + 
        //   CppAD::pow(vars[y_start + j*N + i + 1] - vars[y_start + j*N + i], 2));          
        // }

        // 惩罚终点约束
        fg[0] += W_FINAL * CppAD::pow(vars[x_start + j*N + N - 1] - plan[nu][N-1][0], 2);
        fg[0] += W_FINAL * CppAD::pow(vars[y_start + j*N + N - 1] - plan[nu][N-1][1], 2); 
        // Initial constraints
        fg[0] += W_THETA * CppAD::pow(vars[theta_start + j * N + N - 1] - plan[nu][N-1][2], 2);
        // fg[0] += W_THETA * CppAD::pow(CppAD::sin(vars[theta_start + j * N + N - 1]) - CppAD::sin(plan[nu][N-1][2]), 2);
        // fg[0] += W_THETA * CppAD::pow(CppAD::cos(vars[theta_start + j * N + N - 1]) - CppAD::cos(plan[nu][N-1][2]), 2);

        fg[1 + N * j + x_start] = vars[x_start + N * j];
        fg[1 + N * j + y_start] = vars[y_start + N * j];
        fg[1 + N * j + theta_start] = vars[theta_start + N * j];

        // The rest of the constraints
        // kinematic constraints
        for (int i = 0; i < N - 1; i++) {
          // The state at time t+1 .
          AD<double> x1 = vars[x_start + N * j + i + 1];
          AD<double> y1 = vars[y_start + N * j + i + 1];
          AD<double> theta1 = vars[theta_start + N * j + i + 1];

          // The state at time t.
          AD<double> x0 = vars[x_start + N * j + i];
          AD<double> y0 = vars[y_start + N * j + i];
          AD<double> theta0 = vars[theta_start + N * j + i];

          // Only consider the actuation at time t.
          AD<double> v0 = vars[v_start + (N-1) * j + i];
          AD<double> omega0 = vars[omega_start + (N-1) * j + i];
          
          fg[2 + x_start + N * j + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * DT);
          fg[2 + y_start + N * j + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * DT);
          if (robot_type[nu] > 0) {
            fg[2 + theta_start + N * j + i] = theta1 - (theta0 + v0 * CppAD::tan(omega0) / 0.65 * DT);
          }
          else {
            fg[2 + theta_start + N * j + i] = theta1 - (theta0 + omega0 * DT);
          }
        } 
      }

      int cont = N * 3 * group_qn + 1;
      int neww, old, tt;
      // for (int i = 0; i < selected_pair_new.size(); i++) // selected_pair_new: <timestep, robot_i, robot_j>
      //   {
      //     tt = selected_pair_new[i].at(0);
      //     for (int k = 0; k < group_qn; k++) {
      //       if(group[group_num][k] == selected_pair_new[i].at(1))
      //         neww = k;
      //       else if (group[group_num][k] == selected_pair_new[i].at(2))
      //         old = k;
      //     } 
      //     // 同一个组内(相同优先级)的碰撞约束
      //     fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - vars[x_start + N * old + tt], 2)
      //             + CppAD::pow(vars[y_start + N * neww + tt] - vars[y_start + N * old + tt], 2);
      //     cont++;
      //   }  
      // 与高优先级机器人的碰撞约束
      for (int i = 0; i < selected_pair_old.size(); i++)
        {
          tt = selected_pair_old[i].at(0);
          for (int k = 0; k < group_qn; k++){
            if (group[group_num][k] == selected_pair_old[i].at(1)){
              neww = k;
              break;
            }
          }
          Rectangle Rectangle_1 = {vars[x_start + N * neww + tt], vars[y_start + N * neww + tt] , 0.8, 1.0, vars[theta_start + N * neww + tt]}; 
          Rectangle Rectangle_2 = {plan[selected_pair_old[i].at(2)][tt][0], plan[selected_pair_old[i].at(2)][tt][1], 0.8, 1.0, plan[selected_pair_old[i].at(2)][tt][2]}; 
          Point vertices_1[4], vertices_2[4];
          calculateRectangleVertices(Rectangle_1, vertices_1);
          calculateRectangleVertices(Rectangle_2, vertices_2);
          for (int rec_1_ind = 0; rec_1_ind < 4; ++rec_1_ind) {
            ADdouble area = 0.0;
            Point A, B, C;
            for (int rec_2_ind = 0; rec_2_ind < 3; ++rec_2_ind) {
              A = {vertices_1[rec_1_ind].x, vertices_1[rec_1_ind].y};
              B = {vertices_2[rec_2_ind].x, vertices_2[rec_2_ind].y};
              C = {vertices_2[rec_2_ind+1].x, vertices_2[rec_2_ind+1].y};
              area += calculateTriangleArea(A, B, C);
            }
            B = {vertices_2[0].x, vertices_2[0].y};
            C = {vertices_2[3].x, vertices_2[3].y};
            area += calculateTriangleArea(A, B, C);
            fg[cont++] = area;
          }
          for (int rec_2_ind = 0; rec_2_ind < 4; ++rec_2_ind) {
            ADdouble area = 0.0;
            Point A, B, C;
            for (int rec_1_ind = 0; rec_1_ind < 3; ++rec_1_ind) {
              A = {vertices_2[rec_2_ind].x, vertices_2[rec_2_ind].y};
              B = {vertices_1[rec_1_ind].x, vertices_1[rec_1_ind].y};
              C = {vertices_1[rec_1_ind+1].x, vertices_1[rec_1_ind+1].y};
              area += calculateTriangleArea(A, B, C);
            }
            B = {vertices_1[0].x, vertices_1[0].y};
            C = {vertices_1[3].x, vertices_1[3].y};
            area += calculateTriangleArea(A, B, C);
            fg[cont++] = area;
          }
          // 与高优先级机器人的碰撞约束
          // for (int vertex_ind = 0; vertex_ind < 8; vertex_ind++) {
          //   fg[cont] = CppAD::cos(vars[theta_start + N * neww + tt] ) * CppAD::pow(vars[x_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][0], 2)
          //           + CppAD::sin(vars[theta_start + N * neww + tt] ) * CppAD::pow(vars[y_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][1], 2);
          //   cont++;
          // }        
          // cont++;
        }

      // for (int j = 0; j < group_qn; j++) {    
      //   for (int i = 0; i < N; j++) {
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8] - vars[x_start + N * j + i] - 0.4 * CppAD::cos(vars[theta_start + N * j + i]) + 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 1] - vars[x_start + N * j + i] - 0.4 * CppAD::sin(vars[theta_start + N * j + i]) - 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 2] - vars[x_start + N * j + i] + 0.4 * CppAD::cos(vars[theta_start + N * j + i]) + 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 3] - vars[x_start + N * j + i] + 0.4 * CppAD::sin(vars[theta_start + N * j + i]) - 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 4] - vars[x_start + N * j + i] - 0.4 * CppAD::cos(vars[theta_start + N * j + i]) - 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 5] - vars[x_start + N * j + i] - 0.4 * CppAD::sin(vars[theta_start + N * j + i]) + 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 6] - vars[x_start + N * j + i] + 0.4 * CppAD::cos(vars[theta_start + N * j + i]) - 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
      //       fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 7] - vars[x_start + N * j + i] + 0.4 * CppAD::sin(vars[theta_start + N * j + i]) + 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
      //   }
      // }
    }
};


class MPCPlanner {
public:
    std_msgs::Float64MultiArray msgs_traj_info;
    std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;
    std::vector<double> robot_w_list;

    MPCPlanner(
               std::vector<heterogeneous_formation_controller::Constraints> _constraint_set,
               std::shared_ptr<Corridor> _corridor_obj,
               std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
               SwarmPlanning::Mission _mission,
               SwarmPlanning::Param _param,
               std::vector<double>& _theta_set)
            : constraint_set(std::move(_constraint_set)),
              corridor_obj(std::move(_corridor_obj)),
              initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              mission(std::move(_mission)),
              param(std::move(_param)),
              theta_set(_theta_set)

    {
        M = initTrajPlanner_obj.get()->T.size()-1; // the number of segments

        outdim = 3; // the number of outputs (x,y,z)

        T = initTrajPlanner_obj.get()->T;
        initTraj = initTrajPlanner_obj.get()->initTraj;
        // SFC = corridor_obj.get()->SFC;

        int i,j,k;

        double dx;
        double dy;
        double dis;
        

        for (i = 0; i < qn; i++){
            for(j = 0;j < M; j++){

                dx = (initTraj[i][j+1].x() - initTraj[i][j].x()) / 5;
                dy = (initTraj[i][j+1].y() - initTraj[i][j].y()) / 5;
                dis = 25 * (dx*dx + dy*dy);
                 

                for (k = 0; k < 5; k++){
                    plan[i][5*j+k][0] = initTraj[i][j].x() + dx*k;
                    plan[i][5*j+k][1] = initTraj[i][j].y() + dy*k;
                    // 每个时刻的速度
                    if (dis < 0.1)
                    {
                      plan[i][5*j+k][3] = 0;
                    }
                    if (dis<1.1){
                      plan[i][5*j+k][3] = MAXV*0.7;
                    }
                    else{
                      plan[i][5*j+k][3] = MAXV;
                    }
                }
            }
            plan[i][5*M][0] = initTraj[i][M].x();
            plan[i][5*M][1] = initTraj[i][M].y();
        }

        N = M*5+1;
        // 配准朝向角
        for (int qi = 0; qi < qn; qi++)
        {
          for (int next = 5; next < N; next += 5){
            if ((plan[qi][next][1] == plan[qi][0][1]) && (plan[qi][next][0] == plan[qi][0][0]))
              continue;
            else{
              plan[qi][0][2] = atan2(plan[qi][next][1] - plan[qi][0][1],plan[qi][next][0] - plan[qi][0][0]);
              break;
            }
        }
      }

        for (int qi = 0; qi < qn; qi++)
        {
          for (int next = N - 1 - 5; next < N; next -= 5){
            if ((plan[qi][next][1] == plan[qi][N-1][1]) && (plan[qi][next][0] == plan[qi][N-1][0]))
              continue;
            else{
              plan[qi][N-1][2] = atan2(plan[qi][N-1][1] - plan[qi][next][1],plan[qi][N-1][0] - plan[qi][next][0]);
              break;
            }
        }
      }

      double angle;

      for (i = 0; i < qn; i++){
        for(j = 0; j < M; j++){

          dx = initTraj[i][j+1].x() - initTraj[i][j].x();
          dy = initTraj[i][j+1].y() - initTraj[i][j].y();
          dis = dx*dx+dy*dy;
          
          if (j > 0){
            if (dis > 0.1){
              angle = atan2(dy, dx);
              if (angle - plan[i][5*(j-1)][2] > PI)
                angle = angle - 2 * PI;
              else if(plan[i][5*(j-1)][2] - angle > PI)
                angle = angle + 2 * PI;
            }
              else angle = plan[i][5*(j-1)][2];
          }
          else {angle = plan[i][0][2];}

          for (k = 0; k < 5; k++){
            if (angle < -M_PI) {
                while (angle < -M_PI) {
                    angle += 2 * M_PI;
                }
            }
            else if (angle > M_PI) {
                while (angle > M_PI) {
                    angle -= 2 * M_PI;
                }
            }            
            plan[i][5*j + k][2] = angle;
          }
        }
      }
      // plan[~][~][4]存放角速度
      for (i = 0; i < qn; i++){
        for(j = 0; j < M; j++){
          for (k = 0; k < 5; k++){
              plan[i][5*j+k][4] = (plan[i][5*(j+1)][2] - plan[i][5*j][2])*0.2 / DT;
            }
          }
      }

      for (i = 0; i < qn; i++) {
        plan[i][N-1][2] = 0;
      }
      // plan[0][N-1][2] = -1.5707963267948966; plan[1][N-1][2] = 0; plan[2][N-1][2] = 0; plan[3][N-1][2] = 1.5707963267948966; plan[4][N-1][2] = 1.5707963267948966;
      // plan[5][N-1][2] = 0; plan[6][N-1][2] = 0; plan[7][N-1][2] = 0; plan[8][N-1][2] = 0; plan[9][N-1][2] = 1.5707963267948966;
      // plan[10][N-1][2] = -1.5707963267948966; plan[11][N-1][2] = -1.5707963267948966; plan[12][N-1][2] = 1.5707963267948966; plan[13][N-1][2] = 1.5707963267948966;; plan[14][N-1][2] = 1.5707963267948966;
      // plan[0][N-1][2] = 0.0; plan[1][N-1][2] = 0.0; plan[2][N-1][2] = 0.78539816339744828; plan[3][N-1][2] = 0.78539816339744828; plan[4][N-1][2] = 1.5707963267948966;
      // plan[5][N-1][2] = 0; plan[6][N-1][2] = 0.0; plan[7][N-1][2] = 0; plan[8][N-1][2] = 0.0; plan[9][N-1][2] = 0.0;
      // plan[10][N-1][2] = 0.0; plan[11][N-1][2] = 0.78539816339744828; plan[12][N-1][2] = 0.78539816339744828; plan[13][N-1][2] = 0.0;; plan[14][N-1][2] = 0.0;
      // plan[15][N-1][2] = 0.0; plan[16][N-1][2] = 1.5707963267948966;; plan[17][N-1][2] = 1.5707963267948966;
      // plan[0][N-1][2] = -0.78539816339744828; plan[1][N-1][2] = -0.78539816339744828; plan[2][N-1][2] = -0.78539816339744828; plan[3][N-1][2] = -1.5707963267948966; plan[4][N-1][2] =  -0.78539816339744828;
      // plan[5][N-1][2] = -1.5707963267948966; plan[6][N-1][2] = -0.78539816339744828; plan[7][N-1][2] = -0.78539816339744828; plan[8][N-1][2] = -0.78539816339744828; plan[9][N-1][2] = -1.5707963267948966;
      theta_set.resize(plan.size());
      for (int i = 0; i < plan.size(); i++) {      
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(-M_PI, M_PI);
        double random_number = dist(gen);
        theta_set[i] = random_number;
        // plan[i][N-1][2] = random_number;
        plan[i][N-1][2] = 0.0;
      }
      for (i = 0; i < qn; i++) {
        double robot_w = 0;
        for (j = 0; j < plan[i][4].size(); j++) {
          robot_w += fabs(plan[i][4][j]);
        }
        robot_w_list.push_back(robot_w);
      }

    }

  bool UpdateConflictPair(std::vector<std::vector<std::vector<double>>> plan, 
    std::vector<std::vector<std::array<int, 2>>>& relative_pair_updated, const int& current_ind, int& current_ind_conflict) {
      bool is_collide = false;
      relative_pair_updated.clear();
      std::array<int, 2> sa;  // 存放2个int元素的数组
      std::vector <std::array<int, 2>> sb;

      double threshold = 0;
      bool first_conflict_found = false;
      // if (qn <= 8)
      //   threshold = pow(2 * mission.quad_size[0], 2);
      // else
      //   threshold = pow(2 * mission.quad_size[0], 2);
      for (int k = 0; k < N; k++){
        sb.clear();
        // 遍历所有机器人对，找到小于阈值的对(i, j), i << j
        for(int i = 0; i < qn; i++){
          for (int j = i + 1; j < qn; j++)
          {
            // double dis = pow(plan[i][k][0] - plan[j][k][0], 2.0) + pow(plan[i][k][1] - plan[j][k][1], 2.0);
            Rectangle Rectangle_1 = {plan[i][k][0], plan[i][k][1] , 0.8, 1.0, plan[i][k][2]}; 
            Rectangle Rectangle_2 = {plan[j][k][0], plan[j][k][1] , 0.8, 1.0, plan[j][k][2]}; 
            Point vertices_1[4], vertices_2[4];
            calculateRectangleVertices(Rectangle_1, vertices_1);
            calculateRectangleVertices(Rectangle_2, vertices_2);
            for (int rec_1_ind = 0; rec_1_ind < 4; ++rec_1_ind) {
              ADdouble area = 0.0;
              Point A, B, C;
              for (int rec_2_ind = 0; rec_2_ind < 3; ++rec_2_ind) {
                A = {vertices_1[rec_1_ind].x, vertices_1[rec_1_ind].y};
                B = {vertices_2[rec_2_ind].x, vertices_2[rec_2_ind].y};
                C = {vertices_2[rec_2_ind+1].x, vertices_2[rec_2_ind+1].y};
                area += calculateTriangleArea(A, B, C);
              }
              B = {vertices_2[0].x, vertices_2[0].y};
              C = {vertices_2[3].x, vertices_2[3].y};
              area += calculateTriangleArea(A, B, C);
              if (area <= 1.2) {
                is_collide = true;
                break;
              }
            }
            if (!is_collide) {
              for (int rec_2_ind = 0; rec_2_ind < 4; ++rec_2_ind) {
                ADdouble area = 0.0;
                Point A, B, C;
                for (int rec_1_ind = 0; rec_1_ind < 3; ++rec_1_ind) {
                  A = {vertices_2[rec_2_ind].x, vertices_2[rec_2_ind].y};
                  B = {vertices_1[rec_1_ind].x, vertices_1[rec_1_ind].y};
                  C = {vertices_1[rec_1_ind+1].x, vertices_1[rec_1_ind+1].y};
                  area += calculateTriangleArea(A, B, C);
                }
                B = {vertices_1[0].x, vertices_1[0].y};
                C = {vertices_1[3].x, vertices_1[3].y};
                area += calculateTriangleArea(A, B, C);
                if (area <= 1.2) {
                  is_collide = true;
                  break;
                }
              }
            }
            if (is_collide) {
              sb.emplace_back(sa = {i, j});
              if (i == current_ind && !first_conflict_found) {
                robot_constraints[i].emplace(j);
                robot_constraints[j].emplace(i);
                current_ind_conflict = j;
                first_conflict_found = true;
              }
              is_collide = false;
            }
          }
        }  
        relative_pair_updated.emplace_back(sb);
      }   
      return first_conflict_found;
  }

  void UpdateCost(const std::vector<std::vector<std::vector<double>>>& plan, const std::vector<double>& robot_cost_set,
    const int& current_ind) {
    int costMapLength;
    if (!initial_traj_get) {
      for (int i = 0; i < plan.size(); i++) {
        for (int j = 1; j < N; j++) {
          traj_length_set[i] += hypot(plan[i][j][0] - plan[i][j-1][0], plan[i][j][1] - plan[i][j-1][1]);
        }
      }
    }
    else {
      for (int i = 1; i < N; i++) {
        traj_length_set[current_ind] += hypot(plan[current_ind][i][0] - plan[current_ind][i-1][0], plan[current_ind][i][1] - plan[current_ind][i-1][1]);
      }      
    }
    double traj_length_all = 0.0, cost_total = 0.0;
    for (int i = 0; i < traj_length_set.size(); i++) {
      traj_length_all += traj_length_set[i];
      cost_total += robot_cost_set[i];
    }
    if (initial_traj_get) {
      costMapLength = costMap.size();
    }
    else {
      costMapLength = plan.size();
    }
    for (int i = 0; i < costMapLength; i++) {
      int newIndex;
      if (initial_traj_get) {
        newIndex = costMap[i].index;
      }
      else {
        newIndex = i;
      }
      auto it = std::find_if(costMap.begin(), costMap.end(), [newIndex](const RobotCost& robot_cost) { return robot_cost.index == newIndex; });
      if (it != costMap.end()) {
        costMap.erase(it);
      }
      costMap.emplace_back(newIndex, traj_length_set[newIndex]/traj_length_all + robot_cost_set[newIndex]/cost_total + robot_type[newIndex]);
    }
    std::sort(costMap.begin(), costMap.end(), compareCost);
    initial_traj_get = true;

  }

  bool Solve(int index) {
    bool ok = true;
    // vector<double>
    typedef CPPAD_TESTVECTOR(double) Dvector;
    // diff_count = 0;
    // for (int j = 0; j < group_qn; j++) {
    //   qi = group[group_num][j];
    //   if (robot_type[qi] < 1) {
    //     diff_count++;
    //   }
    // }
    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    size_t n_vars = (N * 3 + (N - 1) * 2) * group_qn;
    // size_t n_vars = (N * 3 + (N - 1) * 2) * group_qn + N * 4 * 2 * group_qn;
    // Set the number of constraints
    // size_t n_constraints = N * 3 * group_qn + 2 * group_qn + 1 * diff_count;
    size_t n_constraints = N * 3 * group_qn;
    // size_t all_constraints = n_constraints + qn*(qn-1)*int(N/2)/2;
    size_t all_constraints = n_constraints + selected_pair_new.size() + 8 * selected_pair_old.size();


    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set lower and upper limits for variables.
    for (int i = 0; i < v_start; i++) {
      vars_lowerbound[i] = -BOUND;
      vars_upperbound[i] = BOUND;
    }

    // diff_count = 0;
    // for (int j = 0; j < group_qn; j++) {
    //   qi = group[group_num][j];
    //   if (robot_type[qi] < 1) {
    //     diff_count++;
    //   }
    // }
    vertice_start = (N * 3 + (N - 1) * 2) * group_qn;
    for (int j = 0; j < group_qn; j++){
      qi = group[group_num][j];
      int count = 0;
      for (int i = 0; i < N; i++) {
        // initial guess using initial states
        vars[x_start + j * N + i] = plan[qi][i][0];
        vars[y_start + j * N + i] = plan[qi][i][1];
        vars[theta_start + j * N + i] = plan[qi][i][2];

        // initial guess using initial control
        if (i < N-1)
        {
          vars[v_start + j * (N - 1) + i] = plan[qi][i][3];
          vars[omega_start + j * (N -1) + i] = plan[qi][i][4];
        }
        
        // if ( i > 1 && i == constraint_set[qi][count + 1].second && count < constraint_set[j].size() - 1){
        //   count++;
        // }

        if (plan[qi][i][0] - constraint_set[qi].corridor_lb(count, 0) > 6.7) {
          vars_lowerbound[x_start + j * N + i] = plan[qi][i][0] - 6.7;
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2] = plan[qi][i][0] - 6.7;
          // }
        }
        else {
          vars_lowerbound[x_start + j * N + i] = constraint_set[qi].corridor_lb(count, 0);
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2] = constraint_set[qi].corridor_lb(count, 0);
          // }
        }
        if (constraint_set[qi].corridor_ub(count, 0) - plan[qi][i][0] > 6.7) {
          vars_upperbound[x_start + j * N + i] = plan[qi][i][0] + 6.7;
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2] = plan[qi][i][0] + 6.7;
          // }
        }
        else {
          vars_upperbound[x_start + j * N + i] = constraint_set[qi].corridor_ub(count, 0);
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2] = constraint_set[qi].corridor_ub(count, 0);
          // }
        }
        if (plan[qi][i][1] - constraint_set[qi].corridor_lb(count, 1) > 6.7) {
          vars_lowerbound[y_start + j * N + i] = plan[qi][i][1] - 6.7;
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2 + 1] = plan[qi][i][1] - 6.7;
          // }
        }
        else {
          vars_lowerbound[y_start + j * N + i] = constraint_set[qi].corridor_lb(count, 1);
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2 + 1] = constraint_set[qi].corridor_lb(count, 1);
          // }
        }
        if (constraint_set[qi].corridor_ub(count, 1) - plan[qi][i][1] > 6.7) {
          vars_upperbound[y_start + j * N + i] = plan[qi][i][1] + 6.7;
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2 + 1] = plan[qi][i][1] + 6.7;
          // }
        }
        else {
          vars_upperbound[y_start + j * N + i] = constraint_set[qi].corridor_ub(count, 1); 
          // for (int ver_ind = 0; ver_ind < 4; ver_ind++) {
          //   vars_lowerbound[vertice_start + (j * N + i) * 8 + ver_ind * 2 + 1] = constraint_set[qi].corridor_ub(count, 1);
          // }
        }
        if (count < constraint_set[qi].corridor_lb.rows()) {
          count++;
        }
      }
    }
    for (int j = 0; j < group_qn; j++) {
      qi = group[group_num][j];
      for (int i = v_start + j * (N - 1); i < v_start + (j + 1) * (N - 1); i++) {
        if (robot_type[qi] > 0) {
          vars_lowerbound[i] = 0;
          vars_upperbound[i] = MAXV;
          if (param.backward_enable)
            vars_lowerbound[i] = -MAXV;
        }
        else {
          vars_lowerbound[i] = 0;
          vars_upperbound[i] = MAXV;
          if (param.backward_enable)
            vars_lowerbound[i] = -MAXV;     
        }
      }
    }

    for (int j = 0; j < group_qn; j++) {
      qi = group[group_num][j];
      for (int i = omega_start + j * (N - 1); i < omega_start + (j + 1) * (N - 1); i++) {
        if (robot_type[qi] > 0) {
          vars_upperbound[i] = MAXPHI;
          vars_lowerbound[i] = -MAXPHI; 
        }
        else {
          vars_upperbound[i] = MAXOMEGA;
          vars_lowerbound[i] = -MAXOMEGA;
        }
      }
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(all_constraints);
    Dvector constraints_upperbound(all_constraints);
    for (int i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
    }
    // 0.2 to 0.3: collsion avoidance distance
    // double inter_collision = 2 * mission.quad_size[0] + 0.2;
    // inter_collision = inter_collision * inter_collision;
    for (int i = n_constraints; i < all_constraints; i++)
    {
      // constraints_lowerbound[i] = inter_collision;
      constraints_lowerbound[i] = 1.2;
      constraints_upperbound[i] = BOUND;
    }

    for (int j = 0; j < group_qn; j ++){
      qi = group[group_num][j];
      double startangle;
      for (int next = 5; next < N; next += 5){
        if ((plan[qi][next][1] == plan[qi][0][1]) & (plan[qi][next][0] == plan[qi][0][0]))
          continue;
        else{
          startangle = atan2(plan[qi][next][1] - plan[qi][0][1], plan[qi][next][0] - plan[qi][0][0]);
          break;
        }
      }
    // start position constraints
      constraints_lowerbound[x_start + N * j] = plan[qi][0][0];
      constraints_lowerbound[y_start + N * j] = plan[qi][0][1];
      constraints_upperbound[x_start + N * j] = plan[qi][0][0];
      constraints_upperbound[y_start + N * j] = plan[qi][0][1];

      if(param.initial_angle)
      {
        constraints_lowerbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
        constraints_upperbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
      }
      else
      { 
        constraints_lowerbound[theta_start + N * j] = startangle;
        constraints_upperbound[theta_start + N * j] = startangle;
      }

    }

    // object that computes objective and constraints
    FG_eval fg_eval;
    // options for IPOPT solver
    std::string options;

    options += "Numeric tol          1e-5\n";
    options += "String linear_solver mumps\n";
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;


    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    cost += solution.obj_value;

    if(!ok){
      ROS_INFO_STREAM("Infeasible Solution: Group_" << group_num);
      return false;
    }


    for (int j = 0; j < group_qn; j++){
      qi = group[group_num][j];
      for (int i = 0; i < N; i++){
          plan[qi][i][0] = solution.x[x_start + j * N + i];
          plan[qi][i][1] = solution.x[y_start + j * N + i];
          plan[qi][i][2] = solution.x[theta_start + j * N + i];
      }
      for (int i = 0; i < N - 1; i++) {
          plan[qi][i][3] = solution.x[v_start + j * (N - 1) + i];
          plan[qi][i][4] = solution.x[omega_start + j * (N - 1) + i];
      }
    }
    robot_cost_set[index] = solution.obj_value;
    return true;
  }

    // selected_pair_old是高优先级的机器人吗？ 是:)
    bool update(bool log, std::vector<std::vector<std::vector<double>>>& plan_set){   
        initial_traj_get = false;
        robot_cost_set.resize(qn, 0.0);
        robot_constraints.resize(qn);
        traj_length_set.resize(plan.size(), 0.0);
        Timer timer_step;
        for (int i = 0; i < mission.quad_type.size(); i++) {
          robot_type.push_back(mission.quad_type[i]);
        }
        geometry_msgs::PoseStamped pp;
        group.clear();
        selected_pair_old.clear();
        // optimization without coordination
        for (int i = 0; i < qn; i++) {
          group.push_back({i});
        }
        group_qn = 1;
        timer_step.reset();
        for (int i = 0; i < group.size(); i++) {
          group_num = i;
          int temp_qn = group[i].size();
          x_start = 0;
          y_start = x_start + temp_qn * N;
          theta_start = y_start + temp_qn * N;
          v_start = theta_start + temp_qn * N;
          omega_start = v_start + temp_qn * (N - 1);          
          Solve(group[i][0]);
        }
        // return true;
        UpdateCost(plan, robot_cost_set, -1);
        while (!costMap.empty()) {
          int current_ind = costMap.back().index;
          int current_ind_conflict;
          if (!UpdateConflictPair(plan, relative_pair_updated, current_ind, current_ind_conflict)) {
            costMap.pop_back();
            ROS_WARN("No conflict found for robot%d!", current_ind);
            continue;
          }
          group.clear();
          group.push_back({current_ind}); 
          group.push_back({current_ind_conflict}); 
          for (int i = 0; i < group.size(); i++) {
            int temp_qn = group[i].size();
            x_start = 0;
            y_start = x_start + temp_qn * N;
            theta_start = y_start + temp_qn * N;
            v_start = theta_start + temp_qn * N;
            omega_start = v_start + temp_qn * (N - 1);
            
            group_num = i; // 组的序列号
            group_qn = temp_qn; // 当前组的机器人个数
            vertice_start = (N * 3 + (N - 1) * 2) * group_qn;

            // selected_pair_new.clear(); // 用于存放同一组相同优先级机器人之间的所有碰撞
            selected_pair_old.clear(); // 用于存放当前组中机器人与其他机器人之间的所有碰撞
            // std::set<int> new_set; // 存放当前组

            // for(int j = 0; j < group_qn; j++)
            //   new_set.insert(group[0][j]);  

            int n1, n2;
            for (int j = 0; j < relative_pair_updated.size(); j++){ // j可以表示为time step
              for (int k = 0; k < relative_pair_updated[j].size(); k++){
                n1 = relative_pair_updated[j][k].at(0);
                n2 = relative_pair_updated[j][k].at(1);

                if (n1 == group[i][0] && std::find(robot_constraints[n1].begin(), robot_constraints[n1].end(), n2) != robot_constraints[n1].end())
                {
                  selected_pair_old.emplace_back(std::array<int, 3>{j, n1, n2});
                }
                else if (n2 == group[i][0] && std::find(robot_constraints[n2].begin(), robot_constraints[n2].end(), n1) != robot_constraints[n2].end())
                {
                  selected_pair_old.emplace_back(std::array<int, 3>{j, n2, n1});
                }
                // else if (new_set.count(n1) == 1 && new_set.count(n2) == 1)
                // {
                //   selected_pair_new.emplace_back(std::array<int, 3>{j, n1, n2});
                // }
              }
            }

            bool group_ok =Solve(current_ind);
            if (group_ok) {
              ROS_INFO_STREAM("Solve robot" << group[i][0] << " success!");
            }
            UpdateCost(plan, robot_cost_set, group[i][0]);
            // 如果规划失败，则将失败的组插入到最高优先级，重新进行规划
            // if ((!group_ok) && (!param.random_group))
            // {
            //   std::vector <int> group_temp = group[i];
            //   group.erase(group.begin() + i);
            //   group.insert(group.begin(), group_temp);
            //   i = -1;
            //   old_set.clear();
            //   ROS_INFO_STREAM("Try again");
            //   continue;
            // }
            // 优化完成后存放优化后了的组的机器人索引值
            // for(int j = 0; j < group_qn; j++)
            //   old_set.insert(group[0][j]);

            ROS_INFO_STREAM("Optimization Success!");
            // ROS_INFO_STREAM("Cost=" << cost);
          }
        }
        ROS_WARN("Formation generation success!");
        timer_step.stop();
        ROS_INFO_STREAM("MPC runtime: " << timer_step.elapsedSeconds());
        double cost_total = 0.0;
        for (int i = 0; i < robot_cost_set.size(); i++) {
          cost_total += robot_cost_set[i];
        }
        ROS_INFO_STREAM("Total cost: " << cost);
        for (int qi = 0; qi < qn; qi++)
        {
          nav_msgs::Path path;
          path.header.frame_id = "map";
          path.header.stamp = ros::Time::now();
          for (int i = 0; i < N; i++)
          {
              pp.pose.position.x = plan[qi][i][0];
              pp.pose.position.y = plan[qi][i][1];
              pp.pose.orientation = tf::createQuaternionMsgFromYaw(plan[qi][i][2]);
              path.poses.push_back(pp);
              // 确定路径长度
              if (fabs(plan[qi][i][0] - plan[qi][N-1][0]) < 0.01){
                  if(fabs(plan[qi][i][1] - plan[qi][N-1][1]) < 0.01){
                      break;
                  }
              }
          }
        way_point_pub[qi].publish(path);
        }
        plan_set = plan;
        return true;
    }

private:
    std::vector<heterogeneous_formation_controller::Constraints> constraint_set;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    SwarmPlanning::Mission mission;
    SwarmPlanning::Param param;
    std::vector<double>& theta_set;

    initTraj_t initTraj;
    std::vector<double> T;
    // SFC_t SFC;

    int M, phi, outdim;

    std::vector<Eigen::MatrixXd> coef;


    void createMsg(){
        std::vector<double> traj_info;
        traj_info.emplace_back(N);
        traj_info.emplace_back(qn);
        traj_info.insert(traj_info.end(), T.begin(), T.end());
        msgs_traj_info.data = traj_info;

        msgs_traj_coef.resize(N);
        for(int qi = 0; qi < N; qi++) {
            std_msgs::MultiArrayDimension rows;
            rows.size = M * (qn + 1);
            msgs_traj_coef[qi].layout.dim.emplace_back(rows);

            std_msgs::MultiArrayDimension cols;
            cols.size = outdim;
            msgs_traj_coef[qi].layout.dim.emplace_back(cols);

            std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
            msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
        }
    }
};

// // /**
// //  * file prioritized_traj_optimization.hpp
// //  * author Weijian Zhang (wxz163@student.bham.ac.uk)
// //  * brief prioritized trajectory optimization using optimal control
// //  * data 2023-11-22
// //  * 
// //  * @copyright Copyroght(c) 2023
// // */
// // #pragma once

// // #include <algorithm>
// // #include <ros/ros.h>
// // #include <iostream>
// // #include <fstream>
// // #include <sstream>
// // #include <stdio.h>
// // #include <string.h>
// // #include <stdlib.h>

// // // ROS
// // #include <std_msgs/Float64MultiArray.h>
// // #include <std_msgs/MultiArrayDimension.h>

// // #include <tf/transform_broadcaster.h>
// // #include <tf/transform_listener.h>
// // #include <geometry_msgs/Pose.h>
// // #include <nav_msgs/Odometry.h>
// // #include <nav_msgs/Path.h>
// // #include <geometry_msgs/Twist.h>
// // #include <nav_msgs/Path.h>
// // #include <geometry_msgs/PoseStamped.h>
// // #include <geometry_msgs/Point.h>
// // #include <geometry_msgs/PointStamped.h>
// // #include <geometry_msgs/PolygonStamped.h>
// // #include <geometry_msgs/PoseWithCovarianceStamped.h>

// // // EIGEN
// // #include <Eigen/Dense>
// // #include <Eigen/Geometry>


// // // Submodules
// // #include <formation_generation/corridor.hpp>
// // #include <formation_generation/init_traj_planner.hpp>
// // #include <formation_generation/mission.hpp>
// // #include <formation_generation/param.hpp>

// // #include "com_fun.h"
// // #include "optimization.h"
// // #include "time.h"
// // #include <cppad/cppad.hpp>
// // #include <cppad/ipopt/solve.hpp>
// // #include <eigen3/Eigen/Core>

// // #include "heterogeneous_formation_controller/optimizer_interface.h"

// // using CppAD::AD;

// // int qi;

// // double cost;

// // int diff_count;

// // int goal_theta_ind = 0;

// // // The solver takes all the state variables and actuator
// // // variables in a singular vector. Thus, we should to establish
// // // when one variable starts and another ends to make our lifes easier.
// // size_t x_start;
// // size_t y_start;
// // size_t theta_start;
// // size_t v_start;
// // size_t omega_start;

// // std::vector<ros::Publisher> way_point_pub;

// // std::vector<std::vector<std::array<int, 2>>> relative_pair; // 存储每个时刻相关的pair
// // std::vector<std::array<int, 3>> selected_pair_new; // 保存特定同一组(相同优先级)中机器人的碰撞约束
// // std::vector<std::array<int, 3>> selected_pair_old; // 保存特定一组机器人中与高优先级碰撞约束
// // std::vector<std::array<int, 3>> relative_3_pair; // 保存从relative_pair选出的triple
// // std::vector<std::array<int, 2>> relative_3_2_pair;
// // std::vector<std::vector<int>> group;
// // int group_num;
// // int group_qn;
// // std::vector<double> robot_type;
// // std::set<int> old_set;

// // class FG_eval {
// //      public:

// //       typedef CPPAD_TESTVECTOR(AD<double>) ADvector;


// //       void operator()(ADvector& fg, const ADvector& vars) {
// //         // MPC implementation
// //         // fg a vector of constraints, x is a vector of constraints.
// //         // NOTE: You'll probably go back and forth between this function and
// //         // the Solver function below.
// //       fg[0] = 0; 
// //       // diff_count = 0;
// //       // goal_theta_ind = 0;
// //       // for (int j = 0; j < group_qn; j++) {
// //       //   qi = group[group_num][j];
// //       //   if (robot_type[qi] < 1) {
// //       //     diff_count++;
// //       //   }
// //       // }
// //       for (int j = 0; j < group_qn; j++){
        
// //         int nu = group[group_num][j];
// //         ROS_INFO_STREAM("Try to solve Robot_" << nu);
// //         // i为智能体的数量，N为智能体的waypoint个数，第三个为状态数
// //         // 惩罚x, y方向上的初始偏离值
// //         for (int i = 1; i < N - 1; i++) {
// //           fg[0] += W_X * CppAD::pow(vars[x_start + j*N + i] - plan[nu][i][0], 2);
// //           fg[0] += W_Y * CppAD::pow(vars[y_start + j*N + i] - plan[nu][i][1], 2); 
// //         }
// //         // 惩罚v, w的初始值
// //         fg[0] += W_DV * CppAD::pow(vars[v_start+j*(N-1)], 2);
// //         fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start+j*(N-1)], 2);

// //         // Minimize the value gap between sequential actuations.
// //         // 使得控制圆滑：i+1时刻与i时刻的控制指令的差值
// //         for (int i = 0; i < N - 2; i++) {
// //           fg[0] += W_DV * CppAD::pow(vars[v_start + j*(N-1) + i + 1] - vars[v_start + j*(N-1)+ i], 2);
// //           fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start + j*(N-1) + i + 1] - vars[omega_start + j*(N-1) + i], 2);
// //         }

// //         // 惩罚终点约束
// //         fg[0] += W_FINAL * CppAD::pow(vars[x_start + j*N + N - 1] - plan[nu][N-1][0], 2);
// //         fg[0] += W_FINAL * CppAD::pow(vars[y_start + j*N + N - 1] - plan[nu][N-1][1], 2); 
// //         fg[0] += W_THETA * CppAD::pow(CppAD::sin(vars[theta_start + j * N + N - 1]) - CppAD::sin(plan[nu][N-1][2]), 2);
// //         fg[0] += W_THETA * CppAD::pow(CppAD::cos(vars[theta_start + j * N + N - 1]) - CppAD::cos(plan[nu][N-1][2]), 2);

// //         // Initial constraints
// //         fg[1 + N * j + x_start] = vars[x_start + N * j];
// //         fg[1 + N * j + y_start] = vars[y_start + N * j];
// //         fg[1 + N * j + theta_start] = vars[theta_start + N * j];

// //         // Final constraints
// //         // int goal_cons_index = N * 3 * group_qn + selected_pair_new.size() + selected_pair_old.size();
// //         // fg[1 + goal_cons_index + 2 * j] = vars[x_start + N * j + N - 1];
// //         // fg[1 + goal_cons_index + 2 * j + 1] = vars[y_start + N * j + N - 1];
// //         // if (robot_type[nu] < 1) {
// //         //   fg[1 + goal_cons_index + 2 * group_qn + goal_theta_ind] = vars[theta_start + N * j + N - 1];
// //         //   goal_theta_ind++;
// //         // }

// //         // The rest of the constraints
// //         // kinematic constraints
// //         for (int i = 0; i < N - 1; i++) {
// //           // The state at time t+1 .
// //           AD<double> x1 = vars[x_start + N * j + i + 1];
// //           AD<double> y1 = vars[y_start + N * j + i + 1];
// //           AD<double> theta1 = vars[theta_start + N * j + i + 1];

// //           // The state at time t.
// //           AD<double> x0 = vars[x_start + N * j + i];
// //           AD<double> y0 = vars[y_start + N * j + i];
// //           AD<double> theta0 = vars[theta_start + N * j + i];

// //           // Only consider the actuation at time t.
// //           AD<double> v0 = vars[v_start + (N-1) * j + i];
// //           AD<double> omega0 = vars[omega_start + (N-1) * j + i];
          
// //           fg[2 + x_start + N * j + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * DT);
// //           fg[2 + y_start + N * j + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * DT);
// //           if (robot_type[nu] > 0) {
// //             fg[2 + theta_start + N * j + i] = theta1 - (theta0 + v0 * CppAD::tan(omega0) / 0.65 * DT);
// //           }
// //           else {
// //             fg[2 + theta_start + N * j + i] = theta1 - (theta0 + omega0 * DT);
// //           }
// //         } 
// //       }

// //       int cont = N * 3 * group_qn + 1;
// //       int neww, old, tt;
// //       for (int i = 0; i < selected_pair_new.size(); i++) // selected_pair_new: <timestep, robot_i, robot_j>
// //         {
// //           tt = selected_pair_new[i].at(0);
// //           for (int k = 0; k < group_qn; k++) {
// //             if(group[group_num][k] == selected_pair_new[i].at(1))
// //               neww = k;
// //             else if (group[group_num][k] == selected_pair_new[i].at(2))
// //               old = k;
// //           } 
// //           // 同一个组内(相同优先级)的碰撞约束
// //           fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - vars[x_start + N * old + tt], 2)
// //                   + CppAD::pow(vars[y_start + N * neww + tt] - vars[y_start + N * old + tt], 2);
// //           cont++;
// //         }  
// //       for (int i = 0; i < selected_pair_old.size(); i++)
// //         {
// //           tt = selected_pair_old[i].at(0);
// //           for (int k = 0; k < group_qn; k++){
// //             if (group[group_num][k] == selected_pair_old[i].at(1)){
// //               neww = k;
// //               break;
// //             }
// //           }
// //           // 与高优先级机器人的碰撞约束
// //           fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][0], 2)
// //                   +CppAD::pow(vars[y_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][1], 2);
// //           cont++;
// //         }
// //     }
// // };


// // class MPCPlanner {
// // public:
// //     std_msgs::Float64MultiArray msgs_traj_info;
// //     std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;
// //     // std::vector<double> robot_w_list;

// //     MPCPlanner(
// //                std::vector<heterogeneous_formation_controller::Constraints> _constraint_set,
// //                std::shared_ptr<Corridor> _corridor_obj,
// //                std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
// //                SwarmPlanning::Mission _mission,
// //                SwarmPlanning::Param _param)
// //             : constraint_set(std::move(_constraint_set)),
// //               corridor_obj(std::move(_corridor_obj)),
// //               initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
// //               mission(std::move(_mission)),
// //               param(std::move(_param))

// //     {
// //         M = initTrajPlanner_obj.get()->T.size()-1; // the number of segments

// //         outdim = 3; // the number of outputs (x,y,z)

// //         T = initTrajPlanner_obj.get()->T;
// //         initTraj = initTrajPlanner_obj.get()->initTraj;
// //         // SFC = corridor_obj.get()->SFC;

// //         int i,j,k;

// //         double dx;
// //         double dy;
// //         double dis;
        

// //         for (i = 0; i < qn; i++){
// //             for(j = 0;j < M; j++){

// //                 dx = (initTraj[i][j+1].x() - initTraj[i][j].x()) / 5;
// //                 dy = (initTraj[i][j+1].y() - initTraj[i][j].y()) / 5;
// //                 dis = 25 * (dx*dx + dy*dy);
                 

// //                 for (k = 0; k < 5; k++){
// //                     plan[i][5*j+k][0] = initTraj[i][j].x() + dx*k;
// //                     plan[i][5*j+k][1] = initTraj[i][j].y() + dy*k;
// //                     // 每个时刻的速度
// //                     if (dis < 0.1)
// //                     {
// //                       plan[i][5*j+k][3] = 0;
// //                     }
// //                     if (dis<1.1){
// //                       plan[i][5*j+k][3] = MAXV*0.7;
// //                     }
// //                     else{
// //                       plan[i][5*j+k][3] = MAXV;
// //                     }
// //                 }
// //             }
// //             plan[i][5*M][0] = initTraj[i][M].x();
// //             plan[i][5*M][1] = initTraj[i][M].y();
// //         }

// //         N = M*5+1;
// //         // 配准朝向角
// //         for (int qi = 0; qi < qn; qi++)
// //         {
// //           for (int next = 5; next < N; next += 5){
// //             if ((plan[qi][next][1] == plan[qi][0][1]) && (plan[qi][next][0] == plan[qi][0][0]))
// //               continue;
// //             else{
// //               plan[qi][0][2] = atan2(plan[qi][next][1] - plan[qi][0][1],plan[qi][next][0] - plan[qi][0][0]);
// //               break;
// //             }
// //         }
// //       }

// //         for (int qi = 0; qi < qn; qi++)
// //         {
// //           for (int next = N - 1 - 5; next < N; next -= 5){
// //             if ((plan[qi][next][1] == plan[qi][N-1][1]) && (plan[qi][next][0] == plan[qi][N-1][0]))
// //               continue;
// //             else{
// //               plan[qi][N-1][2] = atan2(plan[qi][N-1][1] - plan[qi][next][1],plan[qi][N-1][0] - plan[qi][next][0]);
// //               break;
// //             }
// //         }
// //       }

// //       double angle;

// //       for (i = 0; i < qn; i++){
// //         for(j = 0; j < M; j++){

// //           dx = initTraj[i][j+1].x() - initTraj[i][j].x();
// //           dy = initTraj[i][j+1].y() - initTraj[i][j].y();
// //           dis = dx*dx+dy*dy;
          
// //           if (j > 0){
// //             if (dis > 0.1){
// //               angle = atan2(dy, dx);
// //               if (angle - plan[i][5*(j-1)][2] > PI)
// //                 angle = angle - 2 * PI;
// //               else if(plan[i][5*(j-1)][2] - angle > PI)
// //                 angle = angle + 2 * PI;
// //             }
// //               else angle = plan[i][5*(j-1)][2];
// //           }
// //           else {angle = plan[i][0][2];}

// //           for (k = 0; k < 5; k++){
// //             if (angle < -M_PI) {
// //                 while (angle < -M_PI) {
// //                     angle += 2 * M_PI;
// //                 }
// //             }
// //             else if (angle > M_PI) {
// //                 while (angle > M_PI) {
// //                     angle -= 2 * M_PI;
// //                 }
// //             }            
// //             plan[i][5*j + k][2] = angle;
// //           }
// //         }
// //       }
// //       // plan[~][~][4]存放角速度
// //       for (i = 0; i < qn; i++){
// //         for(j = 0; j < M; j++){
// //           for (k = 0; k < 5; k++){
// //               plan[i][5*j+k][4] = (plan[i][5*(j+1)][2] - plan[i][5*j][2])*0.2 / DT;
// //             }
// //           }
// //       }

// //       for (i = 0; i < qn; i++) {
// //         plan[i][N-1][2] = 0;
// //       }
// //       plan[0][N-1][2] = 0; plan[1][N-1][2] = 0; plan[2][N-1][2] = 1.5707963267948966; plan[3][N-1][2] = -1.5707963267948966; plan[4][N-1][2] = 1.5707963267948966;
// //       plan[5][N-1][2] = 0; plan[6][N-1][2] = 0; plan[7][N-1][2] = 1.5707963267948966; plan[8][N-1][2] = -1.5707963267948966; plan[9][N-1][2] = 1.5707963267948966;
// //       plan[10][N-1][2] = 0; plan[11][N-1][2] = 0; plan[12][N-1][2] = 1.5707963267948966; plan[13][N-1][2] = -1.5707963267948966;; plan[14][N-1][2] = 1.5707963267948966;

// //       // for (i = 0; i < qn; i++) {
// //       //   double robot_w = 0;
// //       //   for (j = 0; j < plan[i][4].size(); j++) {
// //       //     robot_w += fabs(plan[i][4][j]);
// //       //   }
// //       //   robot_w_list.push_back(robot_w);
// //       // }

// //       std::array<int, 2> sa;  // 存放2个int元素的数组
// //       std::vector <std::array<int, 2>> sb;

// //       std::array<int, 3> sa3; // 存放3个int元素的数组
// //       std::vector <std::array<int, 3>> sb3;

// //       vector<std::array<int, 2>>::iterator it1, it2;

// //       int count3 = 0;

// //       double threshold = 0;

// //       if (qn <= 8)
// //         threshold = 4;
// //       else
// //         threshold = 6;
// //       for (k = 0; k < N; k++){
// //         sb.clear();
// //         sb3.clear();
// //         // 遍历所有机器人对，找到小于阈值的对(i, j), i << j
// //         for(i = 0; i < qn; i++){
// //           for (j = i + 1; j < qn; j++)
// //           {
// //             dis = pow(plan[i][k][0] - plan[j][k][0], 2.0) + pow(plan[i][k][1] - plan[j][k][1], 2.0);
// //             if (dis < threshold)
// //               sb.emplace_back(sa = {i, j});
// //           }
// //         }  
// //         for (int ii = 0; ii < sb.size(); ii++){
// //           // 遍历大于j+1的机器人
// //           for (int jj = sb[ii].at(1) + 1; jj < qn; jj++){
// //             it1 = find(sb.begin() + ii, sb.end(), sa = {sb[ii].at(0), jj});
// //             // 在高索引的机器人中，是否有与一对中的第1个满足条件的机器人？
// //             if (it1 != sb.end()) {
// //               it2 = find(sb.begin() + ii, sb.end(), sa = {sb[ii].at(1), jj});
// //               // 在高索引的机器人中，是否有与一对中的第2个满足条件的机器人？
// //               if (it2 != sb.end()){
// //                 // sb3.emplace_back(sa3 = {sb[ii].at(0), sb[ii].at(1), jj});
// //                 // 如果有，将原有的一对和新的jj作为triple添加到relative_3_pair中
// //                 relative_3_pair.emplace_back(sa3 = {sb[ii].at(0), sb[ii].at(1), jj});
// //                 count3++;
// //               }
// //             }      
// //           }
// //         }
// //         relative_pair.emplace_back(sb);
// //       }

// //       //std::cout << "Num_conflict=" << count3 << std::endl;

// //     }

// //   bool Solve(int index) {
// //     bool ok = true;
// //     // vector<double>
// //     typedef CPPAD_TESTVECTOR(double) Dvector;
// //     // diff_count = 0;
// //     // for (int j = 0; j < group_qn; j++) {
// //     //   qi = group[group_num][j];
// //     //   if (robot_type[qi] < 1) {
// //     //     diff_count++;
// //     //   }
// //     // }
// //     // Set the number of model variables (includes both states and inputs).
// //     // For example: If the state is a 4 element vector, the actuators is a 2
// //     // element vector and there are 10 timesteps. The number of variables is:
// //     size_t n_vars = (N * 3 + (N - 1) * 2) * group_qn;
// //     // Set the number of constraints
// //     // size_t n_constraints = N * 3 * group_qn + 2 * group_qn + 1 * diff_count;
// //     size_t n_constraints = N * 3 * group_qn;
// //     // size_t all_constraints = n_constraints + qn*(qn-1)*int(N/2)/2;
// //     size_t all_constraints = n_constraints + selected_pair_new.size() + selected_pair_old.size();


// //     // Initial value of the independent variables.
// //     // SHOULD BE 0 besides initial state.
// //     Dvector vars(n_vars);

// //     Dvector vars_lowerbound(n_vars);
// //     Dvector vars_upperbound(n_vars);
// //     // Set lower and upper limits for variables.
// //     for (int i = 0; i < v_start; i++) {
// //       vars_lowerbound[i] = -BOUND;
// //       vars_upperbound[i] = BOUND;
// //     }

// //     // diff_count = 0;
// //     // for (int j = 0; j < group_qn; j++) {
// //     //   qi = group[group_num][j];
// //     //   if (robot_type[qi] < 1) {
// //     //     diff_count++;
// //     //   }
// //     // }
// //     for (int j = 0; j < group_qn; j++){
// //       qi = group[group_num][j];
// //       int count = 0;
// //       for (int i = 0; i < N; i++) {
// //         // initial guess using initial states
// //         vars[x_start + j * N + i] = plan[qi][i][0];
// //         vars[y_start + j * N + i] = plan[qi][i][1];
// //         vars[theta_start + j * N + i] = plan[qi][i][2];

// //         // initial guess using initial control
// //         if (i < N-1)
// //         {
// //           vars[v_start + j * (N - 1) + i] = plan[qi][i][3];
// //           vars[omega_start + j * (N -1) + i] = plan[qi][i][4];
// //         }
        
// //         // if ( i > 1 && i == constraint_set[qi][count + 1].second && count < constraint_set[j].size() - 1){
// //         //   count++;
// //         // }

// //         if (plan[qi][i][0] - constraint_set[qi].corridor_lb(count, 0) > 6.7)
// //           vars_lowerbound[x_start + j * N + i] = plan[qi][i][0] - 6.7;
// //         else
// //           vars_lowerbound[x_start + j * N + i] = constraint_set[qi].corridor_lb(count, 0);
// //         if (constraint_set[qi].corridor_ub(count, 0) - plan[qi][i][0] > 6.7)
// //           vars_upperbound[x_start + j * N + i] = plan[qi][i][0] + 6.7;
// //         else
// //           vars_upperbound[x_start + j * N + i] = constraint_set[qi].corridor_ub(count, 0);
// //         if (plan[qi][i][1] - constraint_set[qi].corridor_lb(count, 1) > 6.7)
// //           vars_lowerbound[y_start + j * N + i] = plan[qi][i][1] - 6.7;
// //         else
// //           vars_lowerbound[y_start + j * N + i] = constraint_set[qi].corridor_lb(count, 1);
// //         if (constraint_set[qi].corridor_ub(count, 1) - plan[qi][i][1] > 6.7)
// //           vars_upperbound[y_start + j * N + i] = plan[qi][i][1] + 6.7;
// //         else
// //           vars_upperbound[y_start + j * N + i] = constraint_set[qi].corridor_ub(count, 1); 
// //         if (count < constraint_set[qi].corridor_lb.rows()) {
// //           count++;
// //         }
// //       }
// //     }

// //     for (int i = v_start; i < omega_start; i++) {
// //       vars_lowerbound[i] = 0;
// //       vars_upperbound[i] = MAXV;
// //       if (param.backward_enable)
// //         vars_lowerbound[i] = -MAXV;
// //     }

// //     for (int j = 0; j < group_qn; j++) {
// //       qi = group[group_num][j];
// //       for (int i = omega_start + j * (N - 1); i < omega_start + (j + 1) * (N - 1); i++) {
// //         if (robot_type[qi] > 0) {
// //           vars_upperbound[i] = MAXPHI;
// //           vars_lowerbound[i] = -MAXPHI; 
// //         }
// //         else {
// //           vars_upperbound[i] = MAXOMEGA;
// //           vars_lowerbound[i] = -MAXOMEGA;
// //         }
// //       }
// //     }

// //     // Lower and upper limits for the constraints
// //     // Should be 0 besides initial state.
// //     Dvector constraints_lowerbound(all_constraints);
// //     Dvector constraints_upperbound(all_constraints);
// //     for (int i = 0; i < n_constraints; i++) {
// //       constraints_lowerbound[i] = 0;
// //       constraints_upperbound[i] = 0;
// //     }
// //     // 0.2 to 0.3: collsion avoidance distance
// //     double inter_collision = 1.3;
// //     inter_collision = inter_collision * inter_collision;
// //     for (int i = n_constraints; i < all_constraints; i++)
// //     {
// //       constraints_lowerbound[i] = inter_collision;
// //       constraints_upperbound[i] = BOUND;
// //     }

// //     for (int j = 0; j < group_qn; j ++){
// //       qi = group[group_num][j];
// //       double startangle;
// //       for (int next = 5; next < N; next += 5){
// //         if ((plan[qi][next][1] == plan[qi][0][1]) & (plan[qi][next][0] == plan[qi][0][0]))
// //           continue;
// //         else{
// //           startangle = atan2(plan[qi][next][1] - plan[qi][0][1], plan[qi][next][0] - plan[qi][0][0]);
// //           break;
// //         }
// //       }
// //     // start position constraints
// //       constraints_lowerbound[x_start + N * j] = plan[qi][0][0];
// //       constraints_lowerbound[y_start + N * j] = plan[qi][0][1];
// //       constraints_upperbound[x_start + N * j] = plan[qi][0][0];
// //       constraints_upperbound[y_start + N * j] = plan[qi][0][1];

// //       if(param.initial_angle)
// //       {
// //         constraints_lowerbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
// //         constraints_upperbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
// //       }
// //       else
// //       { 
// //         constraints_lowerbound[theta_start + N * j] = startangle;
// //         constraints_upperbound[theta_start + N * j] = startangle;
// //       }

// //     }

// //     // goal_theta_ind = 0;
// //     // for (int j = 0; j < group_qn; j ++){
// //     //   qi = group[group_num][j];
// //     //   double endangle;
// //     //   for (int next = N - 6; next > -1; next -= 5){
// //     //     if ((plan[qi][next][1] == plan[qi][N - 1][1]) & (plan[qi][next][0] == plan[qi][N - 1][0]))
// //     //       continue;
// //     //     else{
// //     //       endangle = atan2(plan[qi][N - 1][1] - plan[qi][next][1], plan[qi][N - 1][0] - plan[qi][next][0]);
// //     //       break;
// //     //     }
// //     //   }
// //     // // goal position constraints
// //     //   int goal_cons_index = constraints_lowerbound.size() - group_qn * 2 - diff_count * 1;
// //     //   constraints_lowerbound[goal_cons_index + 2 * j] = plan[qi][N - 1][0];
// //     //   constraints_lowerbound[goal_cons_index + 2 * j + 1] = plan[qi][N - 1][1];
// //     //   constraints_upperbound[goal_cons_index + 2 * j] = plan[qi][N - 1][0];
// //     //   constraints_upperbound[goal_cons_index + 2 * j + 1] = plan[qi][N - 1][1];
// //     //   if (robot_type[qi] < 1) {
// //     //     if(param.initial_angle)
// //     //     {
// //     //       constraints_lowerbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = mission.goalState[qi][2] * PI / 2;
// //     //       constraints_upperbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = mission.goalState[qi][2] * PI / 2;
// //     //     }
// //     //     else
// //     //     {
// //     //       constraints_lowerbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = endangle;
// //     //       constraints_upperbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = endangle;
// //     //     }
// //     //     goal_theta_ind++;
// //     //   }

// //     // }

// //     // object that computes objective and constraints
// //     FG_eval fg_eval;
// //     // options for IPOPT solver
// //     std::string options;

// //     options += "Numeric tol          1e-5\n";
// //     options += "String linear_solver mumps\n";
// //     // Uncomment this if you'd like more print information
// //     options += "Integer print_level  0\n";
// //     // NOTE: Setting sparse to true allows the solver to take advantage
// //     // of sparse routines, this makes the computation MUCH FASTER. If you
// //     // can uncomment 1 of these and see if it makes a difference or not but
// //     // if you uncomment both the computation time should go up in orders of
// //     // magnitude.
// //     options += "Sparse  true        forward\n";
// //     options += "Sparse  true        reverse\n";
// //     // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
// //     // Change this as you see fit.

// //     // place to return solution
// //     CppAD::ipopt::solve_result<Dvector> solution;


// //     // solve the problem
// //     CppAD::ipopt::solve<Dvector, FG_eval>(
// //         options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
// //         constraints_upperbound, fg_eval, solution);

// //     // Check some of the solution values
// //     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
// //     cost += solution.obj_value;

// //     if(!ok){
// //       ROS_INFO_STREAM("Infeasible Solution: Group_" << group_num);
// //       return false;
// //     }


// //     for (int j = 0; j < group_qn; j++){
// //       qi = group[group_num][j];
// //       for (int i = 0; i < N; i++){
// //           plan[qi][i][0] = solution.x[x_start + j * N + i];
// //           plan[qi][i][1] = solution.x[y_start + j * N + i];
// //           plan[qi][i][2] = solution.x[theta_start + j * N + i];
// //       }
// //       for (int i = 0; i < N - 1; i++) {
// //           plan[qi][i][3] = solution.x[v_start + j * (N - 1) + i];
// //           plan[qi][i][4] = solution.x[omega_start + j * (N - 1) + i];
// //       }
// //     }

// //     return true;
// //   }

// //     // selected_pair_old是高优先级的机器人吗？ 是:)
// //     bool update(bool log, std::vector<std::vector<std::vector<double>>>& plan_set){
// //         // struct RobotData {
// //         //   int index;   
// //         //   int label;   
// //         //   double w;  
// //         //   RobotData(int idx, int lbl, double weight) : index(idx), label(lbl), w(weight) {}
// //         //   bool operator<(const RobotData& other) const {
// //         //     if (label == other.label) {
// //         //         return w > other.w;
// //         //     }
// //         //     return label > other.label;
// //         //   }
// //         // };       
// //         for (int i = 0; i < mission.quad_type.size(); i++) {
// //           robot_type.push_back(mission.quad_type[i]);
// //         }
// //         geometry_msgs::PoseStamped pp;

// //         if(param.random_group) {
// //           group.clear();
// //           for (int i = 0; i < 4; i++)
// //           {
// //             for(int j = i; j < qn; j += 4)
// //             {
// //               group.emplace_back(std::vector<int>{j});
// //               std::cout << "Group_" << group.size() << ": " << j << std::endl;
// //             }
// //           }
// //         }
// //         //------------------ grouping starts -----------------------
// //         else {
// //           while(relative_3_pair.size() != 0 || relative_3_2_pair.size() != 0)
// //           {

// //             std::array<int, 3> small_group;
// //             std::array<int, 2> small_group2;
// //             std::array<int, 3> temp;
// //             std::array<int, 2> temp2;
// //             std::map<std::array<int, 3>, int> m;  // 存放triple<3 robots, count>
// //             std::map<std::array<int, 2>, int> m2; // 存放pair<2 robots, count>
// //             // m中存放triple和相应的出现碰撞次数
// //             for(int i = 0; i < relative_3_pair.size(); i++)
// //             {
// //               m[relative_3_pair[i]]++;
// //               if(m[relative_3_pair[i]] > m[small_group])
// //               {
// //                 small_group = relative_3_pair[i];
// //               }
// //             }

// //             for(int i = 0; i < relative_3_2_pair.size(); i++)
// //             {
// //               m2[relative_3_2_pair[i]]++;
// //               if(m2[relative_3_2_pair[i]] > m2[small_group2])
// //               {
// //                 small_group2 = relative_3_2_pair[i];
// //               }
// //             }

// //             // --------------------------- 3-case --------------------------
// //             if(m[small_group] >= m2[small_group2])
// //             {
// //               if(param.log)
// //               // display all members of triple group 
// //               std::cout << "Group_" << group.size() << ": " << small_group.at(0) << ", " << small_group.at(1) << ", "
// //               << small_group.at(2) << std::endl;

// //               std::vector<int> t = {small_group.at(0), small_group.at(1), small_group.at(2)};
// //               // std::vector < std::vector<int> > group;
// //               // 添加一组到group
// //               group.emplace_back(t);

// //               std::vector<std::array<int, 3>>::iterator it;

// //               for(it = relative_3_pair.begin(); it != relative_3_pair.end();)
// //               {
// //                 int erase = 0;
// //                 int num = 0; // 用于判断哪几位包含在了triple中
// //                 temp = *it;
// //                 for(int i = 0; i < 3; i++)
// //                 {
// //                   for (int j = 0; j < 3; j++)
// //                   {
// //                     if(temp.at(i) == small_group.at(j))
// //                     {
// //                       erase++;
// //                       num = i;
// //                       j = 3; // 为了跳出循环
// //                     }
// //                   }
// //                 }
// //                 if (erase == 1) // 有一个机器人相同，其余两个将其加入到relative_3_2_pair
// //                 {
// //                   if(num == 0) // 0表示只有第0位包含在triple中，将后两位划分为double组
// //                     relative_3_2_pair.emplace_back(temp2 = {temp.at(1), temp.at(2)});
// //                   else if (num == 1) // 1表示只有第1位包含在triple中，将另外两个划分出为double组
// //                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(2)});
// //                   else
// //                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(1)});
// //                 }

// //                 if (erase==0) // 如果没有找到包含原有triple的triple，则继续遍历，否则清除这个triple
// //                   ++it;
// //                 else
// //                   it = relative_3_pair.erase(it);
// //               }
// //               // 以下部分没什么必要？
// //               std::vector<std::array<int, 2>>::iterator iter;

// //               for(iter = relative_3_2_pair.begin(); iter != relative_3_2_pair.end();)
// //               {
// //                 temp2 = *iter;
// //                 int erase = 0;
// //                 for(int i = 0; i < 2; i++)
// //                 {
// //                   for (int j = 0; j < 3; j++)
// //                   {
// //                     if(temp2.at(i) == small_group.at(j))
// //                     {
// //                       erase = 1;
// //                       i = 2; j = 3; // 为了跳出循环
// //                     }
// //                   }
// //                 }
// //                 if (erase == 0)
// //                   ++iter;
// //                 else
// //                   iter = relative_3_2_pair.erase(iter);
// //               }
// //             }
// //             // ------------------------ 2-case ---------------------
// //             else
// //             {
// //               if (param.log)
// //               std::cout << "Group_" << group.size() << ": " << small_group2.at(0) << ", " << small_group2.at(1) << std::endl;

// //               std::vector<int> t = {small_group2.at(0), small_group2.at(1)};

// //               group.emplace_back(t);
              
// //               std::vector <std::array<int, 2>>::iterator iter;

// //               for(iter = relative_3_2_pair.begin(); iter != relative_3_2_pair.end();)
// //               {
// //                 temp2 =* iter;
// //                 int erase = 0;
// //                 for(int i = 0; i < 2; i++)
// //                 {
// //                   for (int j = 0; j < 2; j++)
// //                   {
// //                     if(temp2.at(i) == small_group2.at(j)){
// //                       erase = 1;
// //                       i = 2; j = 2;
// //                     }
// //                   }
// //                 }
// //                 if (erase == 0)
// //                   ++iter;
// //                 else
// //                   iter = relative_3_2_pair.erase(iter);
// //               }

// //               std::vector < std::array<int, 3> >::iterator it;

// //               for(it = relative_3_pair.begin(); it != relative_3_pair.end();)
// //               {
// //                 int erase = 0;
// //                 int num = 0;
// //                 temp =* it;
// //                 for(int i = 0; i < 3; i++)
// //                 {
// //                   for (int j = 0; j < 2; j++)
// //                   {
// //                     if(temp.at(i) == small_group2.at(j))
// //                     {
// //                       erase++;
// //                       num = i;
// //                       j = 2;
// //                     }
// //                   }
// //                 }

// //                 if (erase == 1)
// //                 {
// //                   if(num == 0)
// //                     relative_3_2_pair.emplace_back(temp2 = {temp.at(1), temp.at(2)});
// //                   else if (num == 1)
// //                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(2)});
// //                   else
// //                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(1)});
// //                 }

// //                 if (erase == 0)
// //                   ++it;
// //                 else
// //                   it = relative_3_pair.erase(it);
// //               }
// //             }
// //           }
// //           // further grouping
// //           // std::vector<RobotData> robotList;
// //           for (int i = 0; i < qn; i++)
// //           {
// //             bool ingroup = 0;
// //             for(int j = 0; j < group.size(); j++)
// //             {
// //               for(int k = 0; k < group[j].size(); k++)
// //               {
// //                 if(i == group[j][k])
// //                 {
// //                   ingroup = 1;
// //                   break;
// //                 }
// //               }
// //               if(ingroup)
// //                 break;
// //             }
            

// //             if(!ingroup)
// //             {
// //               group.emplace_back(std::vector<int>{i});
// //               if(param.log)
// //               std::cout << "Group_" << group.size() << ": " << i << std::endl;
// //               // robotList.push_back(RobotData(i, robot_type[i], robot_w_list[i]));
// //             }
// //           }
// //           // std::sort(robotList.begin(), robotList.end());
// //           // for (const RobotData& robot : robotList) {
// //           //   group.emplace_back(std::vector<int>{robot.index});
// //           //   if(param.log)
// //           //   std::cout << "Group_" << group.size() << ": " << robot.index << std::endl;
// //           // }
// //         }

// //         for (int i = 0; i < group.size(); i++)
// //         { 
// //             int temp_qn = group[i].size();
// //             x_start = 0;
// //             y_start = x_start + temp_qn * N;
// //             theta_start = y_start + temp_qn * N;
// //             v_start = theta_start + temp_qn * N;
// //             omega_start = v_start + temp_qn * (N - 1);
            
// //             group_num = i; // 组的序列号
// //             group_qn = temp_qn; // 当前组的机器人个数

// //             selected_pair_new.clear(); // 用于存放同一组相同优先级机器人之间的所有碰撞
// //             selected_pair_old.clear(); // 用于存放当前组中机器人与其他机器人之间的所有碰撞
// //             std::set<int> new_set; // 存放当前组

// //             for(int j = 0; j < group_qn; j++)
// //               new_set.insert(group[i][j]);  

// //             int n1, n2;
// //             for (int j = 0; j < relative_pair.size(); j++){ // j可以表示为time step
// //               for (int k = 0;k < relative_pair[j].size(); k++){
// //                 n1 = relative_pair[j][k].at(0);
// //                 n2 = relative_pair[j][k].at(1);

// //                 if (new_set.count(n1) == 1 && old_set.count(n2) == 1)
// //                 {
// //                   selected_pair_old.emplace_back(std::array<int, 3>{j, n1, n2});
// //                 }
// //                 else if (new_set.count(n2) == 1 && old_set.count(n1) == 1)
// //                 {
// //                   selected_pair_old.emplace_back(std::array<int, 3>{j, n2, n1});
// //                 }
// //                 else if (new_set.count(n1) == 1 && new_set.count(n2) == 1)
// //                 {
// //                   selected_pair_new.emplace_back(std::array<int, 3>{j, n1, n2});
// //                 }
// //               }
// //             }

// //             bool group_ok =Solve(i);
// //             // 如果规划失败，则将失败的组插入到最高优先级，重新进行规划
// //             if ((!group_ok) && (!param.random_group))
// //             {
// //               std::vector <int> group_temp = group[i];
// //               group.erase(group.begin() + i);
// //               group.insert(group.begin(), group_temp);
// //               i = -1;
// //               old_set.clear();
// //               ROS_INFO_STREAM("Try again");
// //               continue;
// //             }
// //             // 优化完成后存放优化后了的组的机器人索引值
// //             for(int j = 0;j < group_qn; j++)
// //               old_set.insert(group[i][j]);
// //         }

// //         ROS_INFO_STREAM("Optimization Success!");
// //         ROS_INFO_STREAM("Cost=" << cost);

// //         for (int qi = 0; qi < qn; qi++)
// //         {
// //           nav_msgs::Path path;
// //           path.header.frame_id = "map";
// //           path.header.stamp = ros::Time::now();
// //           for (int i = 0; i < N; i++)
// //           {
// //               pp.pose.position.x = plan[qi][i][0];
// //               pp.pose.position.y = plan[qi][i][1];
// //               pp.pose.orientation = tf::createQuaternionMsgFromYaw(plan[qi][i][2]);
// //               path.poses.push_back(pp);
// //               // 确定路径长度
// //               if (fabs(plan[qi][i][0] - plan[qi][N-1][0]) < 0.01){
// //                   if(fabs(plan[qi][i][1] - plan[qi][N-1][1]) < 0.01){
// //                       break;
// //                   }
// //               }
// //           }
// //         way_point_pub[qi].publish(path);
// //         }
// //         plan_set = plan;
// //         return true;
// //     }

// // private:
// //     std::vector<heterogeneous_formation_controller::Constraints> constraint_set;
// //     std::shared_ptr<Corridor> corridor_obj;
// //     std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
// //     SwarmPlanning::Mission mission;
// //     SwarmPlanning::Param param;

// //     initTraj_t initTraj;
// //     std::vector<double> T;
// //     // SFC_t SFC;

// //     int M, phi, outdim;

// //     std::vector<Eigen::MatrixXd> coef;


// //     void createMsg(){
// //         std::vector<double> traj_info;
// //         traj_info.emplace_back(N);
// //         traj_info.emplace_back(qn);
// //         traj_info.insert(traj_info.end(), T.begin(), T.end());
// //         msgs_traj_info.data = traj_info;

// //         msgs_traj_coef.resize(N);
// //         for(int qi = 0; qi < N; qi++) {
// //             std_msgs::MultiArrayDimension rows;
// //             rows.size = M * (qn + 1);
// //             msgs_traj_coef[qi].layout.dim.emplace_back(rows);

// //             std_msgs::MultiArrayDimension cols;
// //             cols.size = outdim;
// //             msgs_traj_coef[qi].layout.dim.emplace_back(cols);

// //             std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
// //             msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
// //         }
// //     }
// // };

// /**
//  * file prioritized_traj_optimization.hpp
//  * author Weijian Zhang (wxz163@student.bham.ac.uk)
//  * brief prioritized trajectory optimization using optimal control
//  * data 2023-11-22
//  * 
//  * @copyright Copyroght(c) 2023
// */
// #pragma once

// #include <algorithm>
// #include <ros/ros.h>
// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>

// // ROS
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/MultiArrayDimension.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <geometry_msgs/Pose.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/PolygonStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>

// // EIGEN
// #include <Eigen/Dense>
// #include <Eigen/Geometry>


// // Submodules
// #include <formation_generation/corridor.hpp>
// #include <formation_generation/init_traj_planner.hpp>
// #include <formation_generation/mission.hpp>
// #include <formation_generation/param.hpp>

// #include "com_fun.h"
// #include "optimization.h"
// #include "time.h"
// #include <cppad/cppad.hpp>
// #include <cppad/ipopt/solve.hpp>
// #include <eigen3/Eigen/Core>

// #include "heterogeneous_formation_controller/optimizer_interface.h"

// using CppAD::AD;

// int qi;

// double cost;

// int diff_count;

// int goal_theta_ind = 0;

// // The solver takes all the state variables and actuator
// // variables in a singular vector. Thus, we should to establish
// // when one variable starts and another ends to make our lifes easier.
// size_t x_start;
// size_t y_start;
// size_t theta_start;
// size_t v_start;
// size_t omega_start;

// std::vector<ros::Publisher> way_point_pub;

// std::vector<std::vector<std::array<int, 2>>> relative_pair; // 存储每个时刻相关的pair
// std::vector<std::array<int, 3>> selected_pair_new; // 保存特定同一组(相同优先级)中机器人的碰撞约束
// std::vector<std::array<int, 3>> selected_pair_old; // 保存特定一组机器人中与高优先级碰撞约束
// std::vector<std::array<int, 3>> relative_3_pair; // 保存从relative_pair选出的triple
// std::vector<std::array<int, 2>> relative_3_2_pair;
// std::vector<std::vector<int>> group;
// int group_num;
// int group_qn;
// std::vector<double> robot_type;
// std::set<int> old_set;

// class FG_eval {
//      public:

//       typedef CPPAD_TESTVECTOR(AD<double>) ADvector;


//       void operator()(ADvector& fg, const ADvector& vars) {
//         // MPC implementation
//         // fg a vector of constraints, x is a vector of constraints.
//         // NOTE: You'll probably go back and forth between this function and
//         // the Solver function below.
//       fg[0] = 0; 
//       // diff_count = 0;
//       // goal_theta_ind = 0;
//       // for (int j = 0; j < group_qn; j++) {
//       //   qi = group[group_num][j];
//       //   if (robot_type[qi] < 1) {
//       //     diff_count++;
//       //   }
//       // }
//       for (int j = 0; j < group_qn; j++){
        
//         int nu = group[group_num][j];
//         ROS_INFO_STREAM("Try to solve Robot_" << nu);
//         // i为智能体的数量，N为智能体的waypoint个数，第三个为状态数
//         // 惩罚x, y方向上的初始偏离值
//         for (int i = 1; i < N - 1; i++) {
//           fg[0] += W_X * CppAD::pow(vars[x_start + j*N + i] - plan[nu][i][0], 2);
//           fg[0] += W_Y * CppAD::pow(vars[y_start + j*N + i] - plan[nu][i][1], 2); 
//         }
//         // 惩罚v, w的初始值
//         fg[0] += W_DV * CppAD::pow(vars[v_start+j*(N-1)], 2);
//         fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start+j*(N-1)], 2);

//         // Minimize the value gap between sequential actuations.
//         // 使得控制圆滑：i+1时刻与i时刻的控制指令的差值
//         for (int i = 0; i < N - 2; i++) {
//           fg[0] += W_DV * CppAD::pow(vars[v_start + j*(N-1) + i + 1] - vars[v_start + j*(N-1)+ i], 2);
//           fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start + j*(N-1) + i + 1] - vars[omega_start + j*(N-1) + i], 2);
//         }

//         // 惩罚终点约束
//         fg[0] += W_FINAL * CppAD::pow(vars[x_start + j*N + N - 1] - plan[nu][N-1][0], 2);
//         fg[0] += W_FINAL * CppAD::pow(vars[y_start + j*N + N - 1] - plan[nu][N-1][1], 2); 
//         fg[0] += W_THETA * CppAD::pow(CppAD::sin(vars[theta_start + j * N + N - 1]) - CppAD::sin(plan[nu][N-1][2]), 2);
//         fg[0] += W_THETA * CppAD::pow(CppAD::cos(vars[theta_start + j * N + N - 1]) - CppAD::cos(plan[nu][N-1][2]), 2);

//         // Initial constraints
//         fg[1 + N * j + x_start] = vars[x_start + N * j];
//         fg[1 + N * j + y_start] = vars[y_start + N * j];
//         fg[1 + N * j + theta_start] = vars[theta_start + N * j];

//         // Final constraints
//         // int goal_cons_index = N * 3 * group_qn + selected_pair_new.size() + selected_pair_old.size();
//         // fg[1 + goal_cons_index + 2 * j] = vars[x_start + N * j + N - 1];
//         // fg[1 + goal_cons_index + 2 * j + 1] = vars[y_start + N * j + N - 1];
//         // if (robot_type[nu] < 1) {
//         //   fg[1 + goal_cons_index + 2 * group_qn + goal_theta_ind] = vars[theta_start + N * j + N - 1];
//         //   goal_theta_ind++;
//         // }

//         // The rest of the constraints
//         // kinematic constraints
//         for (int i = 0; i < N - 1; i++) {
//           // The state at time t+1 .
//           AD<double> x1 = vars[x_start + N * j + i + 1];
//           AD<double> y1 = vars[y_start + N * j + i + 1];
//           AD<double> theta1 = vars[theta_start + N * j + i + 1];

//           // The state at time t.
//           AD<double> x0 = vars[x_start + N * j + i];
//           AD<double> y0 = vars[y_start + N * j + i];
//           AD<double> theta0 = vars[theta_start + N * j + i];

//           // Only consider the actuation at time t.
//           AD<double> v0 = vars[v_start + (N-1) * j + i];
//           AD<double> omega0 = vars[omega_start + (N-1) * j + i];
          
//           fg[2 + x_start + N * j + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * DT);
//           fg[2 + y_start + N * j + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * DT);
//           if (robot_type[nu] > 0) {
//             fg[2 + theta_start + N * j + i] = theta1 - (theta0 + v0 * CppAD::tan(omega0) / 0.65 * DT);
//           }
//           else {
//             fg[2 + theta_start + N * j + i] = theta1 - (theta0 + omega0 * DT);
//           }
//         } 
//       }

//       int cont = N * 3 * group_qn + 1;
//       int neww, old, tt;
//       for (int i = 0; i < selected_pair_new.size(); i++) // selected_pair_new: <timestep, robot_i, robot_j>
//         {
//           tt = selected_pair_new[i].at(0);
//           for (int k = 0; k < group_qn; k++) {
//             if(group[group_num][k] == selected_pair_new[i].at(1))
//               neww = k;
//             else if (group[group_num][k] == selected_pair_new[i].at(2))
//               old = k;
//           } 
//           // 同一个组内(相同优先级)的碰撞约束
//           fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - vars[x_start + N * old + tt], 2)
//                   + CppAD::pow(vars[y_start + N * neww + tt] - vars[y_start + N * old + tt], 2);
//           cont++;
//         }  
//       for (int i = 0; i < selected_pair_old.size(); i++)
//         {
//           tt = selected_pair_old[i].at(0);
//           for (int k = 0; k < group_qn; k++){
//             if (group[group_num][k] == selected_pair_old[i].at(1)){
//               neww = k;
//               break;
//             }
//           }
//           // 与高优先级机器人的碰撞约束
//           fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][0], 2)
//                   +CppAD::pow(vars[y_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][1], 2);
//           cont++;
//         }
//     }
// };


// class MPCPlanner {
// public:
//     std_msgs::Float64MultiArray msgs_traj_info;
//     std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;
//     // std::vector<double> robot_w_list;

//     MPCPlanner(
//                std::vector<heterogeneous_formation_controller::Constraints> _constraint_set,
//                std::shared_ptr<Corridor> _corridor_obj,
//                std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
//                SwarmPlanning::Mission _mission,
//                SwarmPlanning::Param _param)
//             : constraint_set(std::move(_constraint_set)),
//               corridor_obj(std::move(_corridor_obj)),
//               initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
//               mission(std::move(_mission)),
//               param(std::move(_param))

//     {
//         M = initTrajPlanner_obj.get()->T.size()-1; // the number of segments

//         outdim = 3; // the number of outputs (x,y,z)

//         T = initTrajPlanner_obj.get()->T;
//         initTraj = initTrajPlanner_obj.get()->initTraj;
//         // SFC = corridor_obj.get()->SFC;

//         int i,j,k;

//         double dx;
//         double dy;
//         double dis;
        

//         for (i = 0; i < qn; i++){
//             for(j = 0;j < M; j++){

//                 dx = (initTraj[i][j+1].x() - initTraj[i][j].x()) / 5;
//                 dy = (initTraj[i][j+1].y() - initTraj[i][j].y()) / 5;
//                 dis = 25 * (dx*dx + dy*dy);
                 

//                 for (k = 0; k < 5; k++){
//                     plan[i][5*j+k][0] = initTraj[i][j].x() + dx*k;
//                     plan[i][5*j+k][1] = initTraj[i][j].y() + dy*k;
//                     // 每个时刻的速度
//                     if (dis < 0.1)
//                     {
//                       plan[i][5*j+k][3] = 0;
//                     }
//                     if (dis<1.1){
//                       plan[i][5*j+k][3] = MAXV*0.7;
//                     }
//                     else{
//                       plan[i][5*j+k][3] = MAXV;
//                     }
//                 }
//             }
//             plan[i][5*M][0] = initTraj[i][M].x();
//             plan[i][5*M][1] = initTraj[i][M].y();
//         }

//         N = M*5+1;
//         // 配准朝向角
//         for (int qi = 0; qi < qn; qi++)
//         {
//           for (int next = 5; next < N; next += 5){
//             if ((plan[qi][next][1] == plan[qi][0][1]) && (plan[qi][next][0] == plan[qi][0][0]))
//               continue;
//             else{
//               plan[qi][0][2] = atan2(plan[qi][next][1] - plan[qi][0][1],plan[qi][next][0] - plan[qi][0][0]);
//               break;
//             }
//         }
//       }

//         for (int qi = 0; qi < qn; qi++)
//         {
//           for (int next = N - 1 - 5; next < N; next -= 5){
//             if ((plan[qi][next][1] == plan[qi][N-1][1]) && (plan[qi][next][0] == plan[qi][N-1][0]))
//               continue;
//             else{
//               plan[qi][N-1][2] = atan2(plan[qi][N-1][1] - plan[qi][next][1],plan[qi][N-1][0] - plan[qi][next][0]);
//               break;
//             }
//         }
//       }

//       double angle;

//       for (i = 0; i < qn; i++){
//         for(j = 0; j < M; j++){

//           dx = initTraj[i][j+1].x() - initTraj[i][j].x();
//           dy = initTraj[i][j+1].y() - initTraj[i][j].y();
//           dis = dx*dx+dy*dy;
          
//           if (j > 0){
//             if (dis > 0.1){
//               angle = atan2(dy, dx);
//               if (angle - plan[i][5*(j-1)][2] > PI)
//                 angle = angle - 2 * PI;
//               else if(plan[i][5*(j-1)][2] - angle > PI)
//                 angle = angle + 2 * PI;
//             }
//               else angle = plan[i][5*(j-1)][2];
//           }
//           else {angle = plan[i][0][2];}

//           for (k = 0; k < 5; k++){
//             if (angle < -M_PI) {
//                 while (angle < -M_PI) {
//                     angle += 2 * M_PI;
//                 }
//             }
//             else if (angle > M_PI) {
//                 while (angle > M_PI) {
//                     angle -= 2 * M_PI;
//                 }
//             }            
//             plan[i][5*j + k][2] = angle;
//           }
//         }
//       }
//       // plan[~][~][4]存放角速度
//       for (i = 0; i < qn; i++){
//         for(j = 0; j < M; j++){
//           for (k = 0; k < 5; k++){
//               plan[i][5*j+k][4] = (plan[i][5*(j+1)][2] - plan[i][5*j][2])*0.2 / DT;
//             }
//           }
//       }

//       for (i = 0; i < qn; i++) {
//         plan[i][N-1][2] = 0;
//       }
//       // plan[0][N-1][2] = 0; plan[1][N-1][2] = 0; plan[2][N-1][2] = 1.5707963267948966; plan[3][N-1][2] = -1.5707963267948966; plan[4][N-1][2] = 1.5707963267948966;
//       // plan[5][N-1][2] = 0; plan[6][N-1][2] = 0; plan[7][N-1][2] = 1.5707963267948966; plan[8][N-1][2] = -1.5707963267948966; plan[9][N-1][2] = 1.5707963267948966;
//       // plan[10][N-1][2] = 0; plan[11][N-1][2] = 0; plan[12][N-1][2] = 1.5707963267948966; plan[13][N-1][2] = -1.5707963267948966;; plan[14][N-1][2] = 1.5707963267948966;

//       // for (i = 0; i < qn; i++) {
//       //   double robot_w = 0;
//       //   for (j = 0; j < plan[i][4].size(); j++) {
//       //     robot_w += fabs(plan[i][4][j]);
//       //   }
//       //   robot_w_list.push_back(robot_w);
//       // }

//       std::array<int, 2> sa;  // 存放2个int元素的数组
//       std::vector <std::array<int, 2>> sb;

//       std::array<int, 3> sa3; // 存放3个int元素的数组
//       std::vector <std::array<int, 3>> sb3;

//       vector<std::array<int, 2>>::iterator it1, it2;

//       int count3 = 0;

//       double threshold = 0;

//       if (qn <= 8)
//         threshold = 4;
//       else
//         threshold = 6;
//       for (k = 0; k < N; k++){
//         sb.clear();
//         sb3.clear();
//         // 遍历所有机器人对，找到小于阈值的对(i, j), i << j
//         for(i = 0; i < qn; i++){
//           for (j = i + 1; j < qn; j++)
//           {
//             dis = pow(plan[i][k][0] - plan[j][k][0], 2.0) + pow(plan[i][k][1] - plan[j][k][1], 2.0);
//             if (dis < threshold)
//               sb.emplace_back(sa = {i, j});
//           }
//         }  
//         for (int ii = 0; ii < sb.size(); ii++){
//           // 遍历大于j+1的机器人
//           for (int jj = sb[ii].at(1) + 1; jj < qn; jj++){
//             it1 = find(sb.begin() + ii, sb.end(), sa = {sb[ii].at(0), jj});
//             // 在高索引的机器人中，是否有与一对中的第1个满足条件的机器人？
//             if (it1 != sb.end()) {
//               it2 = find(sb.begin() + ii, sb.end(), sa = {sb[ii].at(1), jj});
//               // 在高索引的机器人中，是否有与一对中的第2个满足条件的机器人？
//               if (it2 != sb.end()){
//                 // sb3.emplace_back(sa3 = {sb[ii].at(0), sb[ii].at(1), jj});
//                 // 如果有，将原有的一对和新的jj作为triple添加到relative_3_pair中
//                 relative_3_pair.emplace_back(sa3 = {sb[ii].at(0), sb[ii].at(1), jj});
//                 count3++;
//               }
//             }      
//           }
//         }
//         relative_pair.emplace_back(sb);
//       }

//       //std::cout << "Num_conflict=" << count3 << std::endl;

//     }

//   bool Solve(int index) {
//     bool ok = true;
//     // vector<double>
//     typedef CPPAD_TESTVECTOR(double) Dvector;
//     // diff_count = 0;
//     // for (int j = 0; j < group_qn; j++) {
//     //   qi = group[group_num][j];
//     //   if (robot_type[qi] < 1) {
//     //     diff_count++;
//     //   }
//     // }
//     // Set the number of model variables (includes both states and inputs).
//     // For example: If the state is a 4 element vector, the actuators is a 2
//     // element vector and there are 10 timesteps. The number of variables is:
//     size_t n_vars = (N * 3 + (N - 1) * 2) * group_qn;
//     // Set the number of constraints
//     // size_t n_constraints = N * 3 * group_qn + 2 * group_qn + 1 * diff_count;
//     size_t n_constraints = N * 3 * group_qn;
//     // size_t all_constraints = n_constraints + qn*(qn-1)*int(N/2)/2;
//     size_t all_constraints = n_constraints + selected_pair_new.size() + selected_pair_old.size();


//     // Initial value of the independent variables.
//     // SHOULD BE 0 besides initial state.
//     Dvector vars(n_vars);

//     Dvector vars_lowerbound(n_vars);
//     Dvector vars_upperbound(n_vars);
//     // Set lower and upper limits for variables.
//     for (int i = 0; i < v_start; i++) {
//       vars_lowerbound[i] = -BOUND;
//       vars_upperbound[i] = BOUND;
//     }

//     // diff_count = 0;
//     // for (int j = 0; j < group_qn; j++) {
//     //   qi = group[group_num][j];
//     //   if (robot_type[qi] < 1) {
//     //     diff_count++;
//     //   }
//     // }
//     for (int j = 0; j < group_qn; j++){
//       qi = group[group_num][j];
//       int count = 0;
//       for (int i = 0; i < N; i++) {
//         // initial guess using initial states
//         vars[x_start + j * N + i] = plan[qi][i][0];
//         vars[y_start + j * N + i] = plan[qi][i][1];
//         vars[theta_start + j * N + i] = plan[qi][i][2];

//         // initial guess using initial control
//         if (i < N-1)
//         {
//           vars[v_start + j * (N - 1) + i] = plan[qi][i][3];
//           vars[omega_start + j * (N -1) + i] = plan[qi][i][4];
//         }
        
//         // if ( i > 1 && i == constraint_set[qi][count + 1].second && count < constraint_set[j].size() - 1){
//         //   count++;
//         // }

//         if (plan[qi][i][0] - constraint_set[qi].corridor_lb(count, 0) > 6.7)
//           vars_lowerbound[x_start + j * N + i] = plan[qi][i][0] - 6.7;
//         else
//           vars_lowerbound[x_start + j * N + i] = constraint_set[qi].corridor_lb(count, 0);
//         if (constraint_set[qi].corridor_ub(count, 0) - plan[qi][i][0] > 6.7)
//           vars_upperbound[x_start + j * N + i] = plan[qi][i][0] + 6.7;
//         else
//           vars_upperbound[x_start + j * N + i] = constraint_set[qi].corridor_ub(count, 0);
//         if (plan[qi][i][1] - constraint_set[qi].corridor_lb(count, 1) > 6.7)
//           vars_lowerbound[y_start + j * N + i] = plan[qi][i][1] - 6.7;
//         else
//           vars_lowerbound[y_start + j * N + i] = constraint_set[qi].corridor_lb(count, 1);
//         if (constraint_set[qi].corridor_ub(count, 1) - plan[qi][i][1] > 6.7)
//           vars_upperbound[y_start + j * N + i] = plan[qi][i][1] + 6.7;
//         else
//           vars_upperbound[y_start + j * N + i] = constraint_set[qi].corridor_ub(count, 1); 
//         if (count < constraint_set[qi].corridor_lb.rows()) {
//           count++;
//         }
//       }
//     }

//     for (int i = v_start; i < omega_start; i++) {
//       vars_lowerbound[i] = 0;
//       vars_upperbound[i] = MAXV;
//       if (param.backward_enable)
//         vars_lowerbound[i] = -MAXV;
//     }

//     for (int j = 0; j < group_qn; j++) {
//       qi = group[group_num][j];
//       for (int i = omega_start + j * (N - 1); i < omega_start + (j + 1) * (N - 1); i++) {
//         if (robot_type[qi] > 0) {
//           vars_upperbound[i] = MAXPHI;
//           vars_lowerbound[i] = -MAXPHI; 
//         }
//         else {
//           vars_upperbound[i] = MAXOMEGA;
//           vars_lowerbound[i] = -MAXOMEGA;
//         }
//       }
//     }

//     // Lower and upper limits for the constraints
//     // Should be 0 besides initial state.
//     Dvector constraints_lowerbound(all_constraints);
//     Dvector constraints_upperbound(all_constraints);
//     for (int i = 0; i < n_constraints; i++) {
//       constraints_lowerbound[i] = 0;
//       constraints_upperbound[i] = 0;
//     }
//     // 0.2 to 0.3: collsion avoidance distance
//     double inter_collision = 1.3;
//     inter_collision = inter_collision * inter_collision;
//     for (int i = n_constraints; i < all_constraints; i++)
//     {
//       constraints_lowerbound[i] = inter_collision;
//       constraints_upperbound[i] = BOUND;
//     }

//     for (int j = 0; j < group_qn; j ++){
//       qi = group[group_num][j];
//       double startangle;
//       for (int next = 5; next < N; next += 5){
//         if ((plan[qi][next][1] == plan[qi][0][1]) & (plan[qi][next][0] == plan[qi][0][0]))
//           continue;
//         else{
//           startangle = atan2(plan[qi][next][1] - plan[qi][0][1], plan[qi][next][0] - plan[qi][0][0]);
//           break;
//         }
//       }
//     // start position constraints
//       constraints_lowerbound[x_start + N * j] = plan[qi][0][0];
//       constraints_lowerbound[y_start + N * j] = plan[qi][0][1];
//       constraints_upperbound[x_start + N * j] = plan[qi][0][0];
//       constraints_upperbound[y_start + N * j] = plan[qi][0][1];

//       if(param.initial_angle)
//       {
//         constraints_lowerbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
//         constraints_upperbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
//       }
//       else
//       { 
//         constraints_lowerbound[theta_start + N * j] = startangle;
//         constraints_upperbound[theta_start + N * j] = startangle;
//       }

//     }

//     // goal_theta_ind = 0;
//     // for (int j = 0; j < group_qn; j ++){
//     //   qi = group[group_num][j];
//     //   double endangle;
//     //   for (int next = N - 6; next > -1; next -= 5){
//     //     if ((plan[qi][next][1] == plan[qi][N - 1][1]) & (plan[qi][next][0] == plan[qi][N - 1][0]))
//     //       continue;
//     //     else{
//     //       endangle = atan2(plan[qi][N - 1][1] - plan[qi][next][1], plan[qi][N - 1][0] - plan[qi][next][0]);
//     //       break;
//     //     }
//     //   }
//     // // goal position constraints
//     //   int goal_cons_index = constraints_lowerbound.size() - group_qn * 2 - diff_count * 1;
//     //   constraints_lowerbound[goal_cons_index + 2 * j] = plan[qi][N - 1][0];
//     //   constraints_lowerbound[goal_cons_index + 2 * j + 1] = plan[qi][N - 1][1];
//     //   constraints_upperbound[goal_cons_index + 2 * j] = plan[qi][N - 1][0];
//     //   constraints_upperbound[goal_cons_index + 2 * j + 1] = plan[qi][N - 1][1];
//     //   if (robot_type[qi] < 1) {
//     //     if(param.initial_angle)
//     //     {
//     //       constraints_lowerbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = mission.goalState[qi][2] * PI / 2;
//     //       constraints_upperbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = mission.goalState[qi][2] * PI / 2;
//     //     }
//     //     else
//     //     {
//     //       constraints_lowerbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = endangle;
//     //       constraints_upperbound[constraints_lowerbound.size() - diff_count + goal_theta_ind] = endangle;
//     //     }
//     //     goal_theta_ind++;
//     //   }

//     // }

//     // object that computes objective and constraints
//     FG_eval fg_eval;
//     // options for IPOPT solver
//     std::string options;

//     options += "Numeric tol          1e-5\n";
//     options += "String linear_solver mumps\n";
//     // Uncomment this if you'd like more print information
//     options += "Integer print_level  0\n";
//     // NOTE: Setting sparse to true allows the solver to take advantage
//     // of sparse routines, this makes the computation MUCH FASTER. If you
//     // can uncomment 1 of these and see if it makes a difference or not but
//     // if you uncomment both the computation time should go up in orders of
//     // magnitude.
//     options += "Sparse  true        forward\n";
//     options += "Sparse  true        reverse\n";
//     // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
//     // Change this as you see fit.

//     // place to return solution
//     CppAD::ipopt::solve_result<Dvector> solution;


//     // solve the problem
//     CppAD::ipopt::solve<Dvector, FG_eval>(
//         options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
//         constraints_upperbound, fg_eval, solution);

//     // Check some of the solution values
//     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
//     cost += solution.obj_value;

//     if(!ok){
//       ROS_INFO_STREAM("Infeasible Solution: Group_" << group_num);
//       return false;
//     }


//     for (int j = 0; j < group_qn; j++){
//       qi = group[group_num][j];
//       for (int i = 0; i < N; i++){
//           plan[qi][i][0] = solution.x[x_start + j * N + i];
//           plan[qi][i][1] = solution.x[y_start + j * N + i];
//           plan[qi][i][2] = solution.x[theta_start + j * N + i];
//       }
//       for (int i = 0; i < N - 1; i++) {
//           plan[qi][i][3] = solution.x[v_start + j * (N - 1) + i];
//           plan[qi][i][4] = solution.x[omega_start + j * (N - 1) + i];
//       }
//     }

//     return true;
//   }

//     // selected_pair_old是高优先级的机器人吗？ 是:)
//     bool update(bool log, std::vector<std::vector<std::vector<double>>>& plan_set){
//         // struct RobotData {
//         //   int index;   
//         //   int label;   
//         //   double w;  
//         //   RobotData(int idx, int lbl, double weight) : index(idx), label(lbl), w(weight) {}
//         //   bool operator<(const RobotData& other) const {
//         //     if (label == other.label) {
//         //         return w > other.w;
//         //     }
//         //     return label > other.label;
//         //   }
//         // };       
//         for (int i = 0; i < mission.quad_type.size(); i++) {
//           robot_type.push_back(mission.quad_type[i]);
//         }
//         geometry_msgs::PoseStamped pp;

//         if(param.random_group) {
//           group.clear();
//           for (int i = 0; i < 4; i++)
//           {
//             for(int j = i; j < qn; j += 4)
//             {
//               group.emplace_back(std::vector<int>{j});
//               std::cout << "Group_" << group.size() << ": " << j << std::endl;
//             }
//           }
//         }
//         //------------------ grouping starts -----------------------
//         else {
//           while(relative_3_pair.size() != 0 || relative_3_2_pair.size() != 0)
//           {

//             std::array<int, 3> small_group;
//             std::array<int, 2> small_group2;
//             std::array<int, 3> temp;
//             std::array<int, 2> temp2;
//             std::map<std::array<int, 3>, int> m;  // 存放triple<3 robots, count>
//             std::map<std::array<int, 2>, int> m2; // 存放pair<2 robots, count>
//             // m中存放triple和相应的出现碰撞次数
//             for(int i = 0; i < relative_3_pair.size(); i++)
//             {
//               m[relative_3_pair[i]]++;
//               if(m[relative_3_pair[i]] > m[small_group])
//               {
//                 small_group = relative_3_pair[i];
//               }
//             }

//             for(int i = 0; i < relative_3_2_pair.size(); i++)
//             {
//               m2[relative_3_2_pair[i]]++;
//               if(m2[relative_3_2_pair[i]] > m2[small_group2])
//               {
//                 small_group2 = relative_3_2_pair[i];
//               }
//             }

//             // --------------------------- 3-case --------------------------
//             if(m[small_group] >= m2[small_group2])
//             {
//               if(param.log)
//               // display all members of triple group 
//               std::cout << "Group_" << group.size() << ": " << small_group.at(0) << ", " << small_group.at(1) << ", "
//               << small_group.at(2) << std::endl;

//               std::vector<int> t = {small_group.at(0), small_group.at(1), small_group.at(2)};
//               // std::vector < std::vector<int> > group;
//               // 添加一组到group
//               group.emplace_back(t);

//               std::vector<std::array<int, 3>>::iterator it;

//               for(it = relative_3_pair.begin(); it != relative_3_pair.end();)
//               {
//                 int erase = 0;
//                 int num = 0; // 用于判断哪几位包含在了triple中
//                 temp = *it;
//                 for(int i = 0; i < 3; i++)
//                 {
//                   for (int j = 0; j < 3; j++)
//                   {
//                     if(temp.at(i) == small_group.at(j))
//                     {
//                       erase++;
//                       num = i;
//                       j = 3; // 为了跳出循环
//                     }
//                   }
//                 }
//                 if (erase == 1) // 有一个机器人相同，其余两个将其加入到relative_3_2_pair
//                 {
//                   if(num == 0) // 0表示只有第0位包含在triple中，将后两位划分为double组
//                     relative_3_2_pair.emplace_back(temp2 = {temp.at(1), temp.at(2)});
//                   else if (num == 1) // 1表示只有第1位包含在triple中，将另外两个划分出为double组
//                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(2)});
//                   else
//                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(1)});
//                 }

//                 if (erase==0) // 如果没有找到包含原有triple的triple，则继续遍历，否则清除这个triple
//                   ++it;
//                 else
//                   it = relative_3_pair.erase(it);
//               }
//               // 以下部分没什么必要？
//               std::vector<std::array<int, 2>>::iterator iter;

//               for(iter = relative_3_2_pair.begin(); iter != relative_3_2_pair.end();)
//               {
//                 temp2 = *iter;
//                 int erase = 0;
//                 for(int i = 0; i < 2; i++)
//                 {
//                   for (int j = 0; j < 3; j++)
//                   {
//                     if(temp2.at(i) == small_group.at(j))
//                     {
//                       erase = 1;
//                       i = 2; j = 3; // 为了跳出循环
//                     }
//                   }
//                 }
//                 if (erase == 0)
//                   ++iter;
//                 else
//                   iter = relative_3_2_pair.erase(iter);
//               }
//             }
//             // ------------------------ 2-case ---------------------
//             else
//             {
//               if (param.log)
//               std::cout << "Group_" << group.size() << ": " << small_group2.at(0) << ", " << small_group2.at(1) << std::endl;

//               std::vector<int> t = {small_group2.at(0), small_group2.at(1)};

//               group.emplace_back(t);
              
//               std::vector <std::array<int, 2>>::iterator iter;

//               for(iter = relative_3_2_pair.begin(); iter != relative_3_2_pair.end();)
//               {
//                 temp2 =* iter;
//                 int erase = 0;
//                 for(int i = 0; i < 2; i++)
//                 {
//                   for (int j = 0; j < 2; j++)
//                   {
//                     if(temp2.at(i) == small_group2.at(j)){
//                       erase = 1;
//                       i = 2; j = 2;
//                     }
//                   }
//                 }
//                 if (erase == 0)
//                   ++iter;
//                 else
//                   iter = relative_3_2_pair.erase(iter);
//               }

//               std::vector < std::array<int, 3> >::iterator it;

//               for(it = relative_3_pair.begin(); it != relative_3_pair.end();)
//               {
//                 int erase = 0;
//                 int num = 0;
//                 temp =* it;
//                 for(int i = 0; i < 3; i++)
//                 {
//                   for (int j = 0; j < 2; j++)
//                   {
//                     if(temp.at(i) == small_group2.at(j))
//                     {
//                       erase++;
//                       num = i;
//                       j = 2;
//                     }
//                   }
//                 }

//                 if (erase == 1)
//                 {
//                   if(num == 0)
//                     relative_3_2_pair.emplace_back(temp2 = {temp.at(1), temp.at(2)});
//                   else if (num == 1)
//                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(2)});
//                   else
//                     relative_3_2_pair.emplace_back(temp2 = {temp.at(0), temp.at(1)});
//                 }

//                 if (erase == 0)
//                   ++it;
//                 else
//                   it = relative_3_pair.erase(it);
//               }
//             }
//           }
//           // further grouping
//           // std::vector<RobotData> robotList;
//           for (int i = 0; i < qn; i++)
//           {
//             bool ingroup = 0;
//             for(int j = 0; j < group.size(); j++)
//             {
//               for(int k = 0; k < group[j].size(); k++)
//               {
//                 if(i == group[j][k])
//                 {
//                   ingroup = 1;
//                   break;
//                 }
//               }
//               if(ingroup)
//                 break;
//             }
            

//             if(!ingroup)
//             {
//               group.emplace_back(std::vector<int>{i});
//               if(param.log)
//               std::cout << "Group_" << group.size() << ": " << i << std::endl;
//               // robotList.push_back(RobotData(i, robot_type[i], robot_w_list[i]));
//             }
//           }
//           // std::sort(robotList.begin(), robotList.end());
//           // for (const RobotData& robot : robotList) {
//           //   group.emplace_back(std::vector<int>{robot.index});
//           //   if(param.log)
//           //   std::cout << "Group_" << group.size() << ": " << robot.index << std::endl;
//           // }
//         }

//         for (int i = 0; i < group.size(); i++)
//         { 
//             int temp_qn = group[i].size();
//             x_start = 0;
//             y_start = x_start + temp_qn * N;
//             theta_start = y_start + temp_qn * N;
//             v_start = theta_start + temp_qn * N;
//             omega_start = v_start + temp_qn * (N - 1);
            
//             group_num = i; // 组的序列号
//             group_qn = temp_qn; // 当前组的机器人个数

//             selected_pair_new.clear(); // 用于存放同一组相同优先级机器人之间的所有碰撞
//             selected_pair_old.clear(); // 用于存放当前组中机器人与其他机器人之间的所有碰撞
//             std::set<int> new_set; // 存放当前组

//             for(int j = 0; j < group_qn; j++)
//               new_set.insert(group[i][j]);  

//             int n1, n2;
//             for (int j = 0; j < relative_pair.size(); j++){ // j可以表示为time step
//               for (int k = 0;k < relative_pair[j].size(); k++){
//                 n1 = relative_pair[j][k].at(0);
//                 n2 = relative_pair[j][k].at(1);

//                 if (new_set.count(n1) == 1 && old_set.count(n2) == 1)
//                 {
//                   selected_pair_old.emplace_back(std::array<int, 3>{j, n1, n2});
//                 }
//                 else if (new_set.count(n2) == 1 && old_set.count(n1) == 1)
//                 {
//                   selected_pair_old.emplace_back(std::array<int, 3>{j, n2, n1});
//                 }
//                 else if (new_set.count(n1) == 1 && new_set.count(n2) == 1)
//                 {
//                   selected_pair_new.emplace_back(std::array<int, 3>{j, n1, n2});
//                 }
//               }
//             }

//             bool group_ok =Solve(i);
//             // 如果规划失败，则将失败的组插入到最高优先级，重新进行规划
//             if ((!group_ok) && (!param.random_group))
//             {
//               std::vector <int> group_temp = group[i];
//               group.erase(group.begin() + i);
//               group.insert(group.begin(), group_temp);
//               i = -1;
//               old_set.clear();
//               ROS_INFO_STREAM("Try again");
//               continue;
//             }
//             // 优化完成后存放优化后了的组的机器人索引值
//             for(int j = 0;j < group_qn; j++)
//               old_set.insert(group[i][j]);
//         }

//         ROS_INFO_STREAM("Optimization Success!");
//         ROS_INFO_STREAM("Cost=" << cost);

//         for (int qi = 0; qi < qn; qi++)
//         {
//           nav_msgs::Path path;
//           path.header.frame_id = "map";
//           path.header.stamp = ros::Time::now();
//           for (int i = 0; i < N; i++)
//           {
//               pp.pose.position.x = plan[qi][i][0];
//               pp.pose.position.y = plan[qi][i][1];
//               pp.pose.orientation = tf::createQuaternionMsgFromYaw(plan[qi][i][2]);
//               path.poses.push_back(pp);
//               // 确定路径长度
//               if (fabs(plan[qi][i][0] - plan[qi][N-1][0]) < 0.01){
//                   if(fabs(plan[qi][i][1] - plan[qi][N-1][1]) < 0.01){
//                       break;
//                   }
//               }
//           }
//         way_point_pub[qi].publish(path);
//         }
//         plan_set = plan;
//         return true;
//     }

// private:
//     std::vector<heterogeneous_formation_controller::Constraints> constraint_set;
//     std::shared_ptr<Corridor> corridor_obj;
//     std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
//     SwarmPlanning::Mission mission;
//     SwarmPlanning::Param param;

//     initTraj_t initTraj;
//     std::vector<double> T;
//     // SFC_t SFC;

//     int M, phi, outdim;

//     std::vector<Eigen::MatrixXd> coef;


//     void createMsg(){
//         std::vector<double> traj_info;
//         traj_info.emplace_back(N);
//         traj_info.emplace_back(qn);
//         traj_info.insert(traj_info.end(), T.begin(), T.end());
//         msgs_traj_info.data = traj_info;

//         msgs_traj_coef.resize(N);
//         for(int qi = 0; qi < N; qi++) {
//             std_msgs::MultiArrayDimension rows;
//             rows.size = M * (qn + 1);
//             msgs_traj_coef[qi].layout.dim.emplace_back(rows);

//             std_msgs::MultiArrayDimension cols;
//             cols.size = outdim;
//             msgs_traj_coef[qi].layout.dim.emplace_back(cols);

//             std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
//             msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
//         }
//     }
// };