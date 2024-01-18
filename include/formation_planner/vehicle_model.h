//
// Created by weijian on 17/11/23.
//
#include "math/pose.h"
#include "math/box2d.h"
#include <tuple>
#include <iostream>

#ifndef SRC_VEHICLE_MODEL_H
#define SRC_VEHICLE_MODEL_H

namespace formation_planner {

class VehicleModel {
public:
  double offset = 2.016733665;
  double offset_angle = atan2(offset / 2, 1.0);
  int vertices = 4;
  /**
   * L_F, front hang length of the ego vehicle (m)
   */
  double front_hang_length = 0.165;

  /**
   * L_W, wheelbase of the ego vehicle (m)
   */
  double wheel_base = 0.65;

  double half_length = (front_hang_length + wheel_base + rear_hang_length) / 2;

  /**
   * L_R, rear hang length of the ego vehicle (m)
   */
  double rear_hang_length = 0.165;

  /**
   * L_B, width of the ego vehicle (m)
   */
  double width = 0.605;

  /**
   * Upper bound of v(t) (m/s)
   */
  double max_velocity = 1.0;
  double max_vel_diff = 1.0;

  /**
   * Lower bound of v(t) (m/s)
   */
  double min_velocity = -1.0;
  double min_vel_diff = -1.0;

  /**
   * Lower and upper bounds of a(t) (m/s^2)
   */
  double max_acceleration = 1.0;
  double max_acc_diff = 2.0;

  /**
   * Upper bound of |\phi(t)| (rad)
   */
  double phi_max = 0.62;
  // double phi_min = 0.62;
  double phi_min = atan2(wheel_base, (wheel_base / tan(phi_max) + offset));
  double omg_max_diff = 1.5;
  
  /**
   * Upper bound of |\omega(t)| (rad/s)
   */
  double omega_max = 0.2;
  double omg_acc_diff = 2.5;

  /**
   * Number of discs covering vehicle footprint
   */
  int n_disc = 2;

  double disc_radius;
  
  std::vector<double> disc_coefficients;

  void InitializeDiscs() {
    double length = wheel_base + rear_hang_length + front_hang_length;
    disc_radius = 0.5 * hypot(length / n_disc, width);

    disc_coefficients.reserve(n_disc);
    for(int i = 0; i < n_disc; i++) {
      disc_coefficients.push_back(double(2 * (i + 1) - 1) / (2 * n_disc) * length - rear_hang_length);
    }
  }

  template<class T>
  std::vector<T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    std::vector<T> result; result.reserve(n_disc * 2);
    for(int i = 0; i < n_disc; i++) {
      result.push_back(x + disc_coefficients[i] * cos(theta));
      result.push_back(y + disc_coefficients[i] * sin(theta));
    }
    return result;
  }

  template<class T>
  std::vector<T> GetFormationCentre(const T &x, const T &y, const T &theta) const {
    std::vector<T> result; result.reserve(2);
    double d_fc = hypot(offset / 2, 1.0);
    result.push_back(x + d_fc * cos(theta - offset_angle));
    result.push_back(y + d_fc * sin(theta - offset_angle));
    return result;
  }

  template<class T>
  std::vector<T> GetVertexPositions(const T &x, const T &y, const T &theta, const T &inflat) const {
    std::vector<T> result; result.reserve(vertices);
    double vert_l_x = x - hypot(rear_hang_length, width / 2) * cos(theta - atan2(width / 2, rear_hang_length));
    double vert_l_y = y - hypot(rear_hang_length, width / 2) * sin(theta - atan2(width / 2, rear_hang_length));
    result.push_back(vert_l_x);
    result.push_back(vert_l_y);
    result.push_back(vert_l_x + (1.2 + inflat * 2.0) * cos(theta));
    result.push_back(vert_l_y + (1.2 + inflat * offset) * sin(theta));
    result.push_back(vert_l_x + hypot(1.2 + inflat * 2.0, 1.2 + inflat * offset) * cos(theta - atan2(1.2 + offset, 1.2 + 2.0)));
    result.push_back(vert_l_y + hypot(1.2 + inflat * 2.0, 1.2 + inflat * offset) * sin(theta - atan2(1.2 + offset, 1.2 + 2.0)));
    result.push_back(vert_l_x + (1.2 + inflat * offset) * cos(M_PI / 2 - theta));
    result.push_back(vert_l_y - (1.2 + inflat * offset) * sin(M_PI / 2 - theta)); 
    return result;
  }

  math::Box2d GenerateBox(const math::Pose &pose) const {
    double length = (wheel_base + rear_hang_length + front_hang_length);
    double distance = length / 2 - rear_hang_length;
    return {pose.extend(distance), pose.theta(), length, width};
  }

};

class FormationConfig {
VehicleModel vehicle;
public:
  math::Pose calculateVertex(const math::Pose& leader_pose, double angle, double radius) {
      math::Pose follower_pos;
      follower_pos.setX(leader_pose.x() + radius * cos(angle));
      follower_pos.setY(leader_pose.y() + radius * sin(angle));
      follower_pos.setTheta(leader_pose.theta());
      return follower_pos;
  }
  void calculateTriangleVertices(const math::Pose& leader_pose, int robot_index,
    math::Pose& follower_pose, double radius) {
      double half_length = (vehicle.front_hang_length + vehicle.wheel_base + vehicle.rear_hang_length) / 2;
      double angle;
      switch (robot_index)
      {
      case 1:
        angle = leader_pose.theta() - M_PI / 2.0;
        follower_pose = calculateVertex(leader_pose, angle, radius);
        break;
      case 2:
        angle = leader_pose.theta() + M_PI / 2.0;   
        follower_pose = calculateVertex(leader_pose, angle, radius);
        break;
      case 3:
        angle = leader_pose.theta(); // 顶点3的角度
        follower_pose = calculateVertex(leader_pose, angle, (vehicle.half_length - vehicle.rear_hang_length + radius));
        break;
      case 4:
        angle = leader_pose.theta() - M_PI;
        follower_pose = calculateVertex(leader_pose, angle, radius - (vehicle.half_length - vehicle.rear_hang_length));
        break;
      default:
        std::cout << "Invalid robot index!" << std::endl;
      }
  }
  /**
   * @param tw_radius radius of diff_drive robot
   * @param cr_offset offset of formation 
  **/
  void calculateMinCircle(double&  min_radius, double tw_radius, double cr_offset) {
    min_radius = std::max(cr_offset + tw_radius, std::sqrt(std::pow(vehicle.half_length, 2) + std::pow(cr_offset + vehicle.width / 2, 2)));
  }
};

class TwoWheelModel {
public: 
  /**
   * Radius of the turtle robot (m)
   */
  // double radius = vehicle->width / 2;
  // 138mm x 178mm x 192mm
  // double radius = 0.113;
  double radius = 0.113;
};

}

#endif //SRC_VEHICLE_MODEL_H
