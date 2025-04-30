//
// Created by weijian on 17/11/23.
//
#include "math/pose.h"
#include "math/box2d.h"
#include <tuple>
#include <iostream>

#ifndef SRC_VEHICLE_MODEL_H
#define SRC_VEHICLE_MODEL_H

namespace heterogeneous_formation_controller {

class VehicleModel {
public:
  double offset = 2;
  // double offset = 2.016733665;
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
  double max_vel_diff = 2.0;

  /**
   * Lower bound of v(t) (m/s)
   */
  double min_velocity = -1.0;
  double min_vel_diff = -1.0;

  /**
   * Lower and upper bounds of a(t) (m/s^2)
   */
  double max_acceleration = 1.0;
  double max_acc_diff = 1.0;

  /**
   * Upper bound of |\phi(t)| (rad)
   */
  // double phi_max = 0.62;
  double phi_max = 0.69;
  double phi_min = 0.69;
  // double phi_min = atan2(wheel_base, (wheel_base / tan(phi_max) + offset));
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

  /**
   * formation constraints
  */
  double min_inter_dis = 0.8;
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
    if (inflat > 1) {
      double offset_ = 2.0;
      double vert_l_x = x - hypot(rear_hang_length, width / 2) * cos(theta - atan2(width / 2, rear_hang_length));
      double vert_l_y = y - hypot(rear_hang_length, width / 2) * sin(theta - atan2(width / 2, rear_hang_length));
      result.push_back(vert_l_x);
      result.push_back(vert_l_y);
      result.push_back(vert_l_x + (1.2 + 2.0) * cos(theta));
      result.push_back(vert_l_y + (1.2 + offset_) * sin(theta));
      result.push_back(vert_l_x + hypot(1.2 + 2.0, 1.2 + offset_) * cos(theta - atan2(1.2 + offset_, 1.2 + 2.0)));
      result.push_back(vert_l_y + hypot(1.2 + 2.0, 1.2 + offset_) * sin(theta - atan2(1.2 + offset_, 1.2 + 2.0)));
      result.push_back(vert_l_x + (1.2 + offset_) * cos(M_PI / 2 - theta));
      result.push_back(vert_l_y - (1.2 + offset_) * sin(M_PI / 2 - theta)); 
      return result;      
    }
    double vert_l_x = x - hypot(rear_hang_length, width / 2) * cos(theta - atan2(width / 2, rear_hang_length));
    double vert_l_y = y - hypot(rear_hang_length, width / 2) * sin(theta - atan2(width / 2, rear_hang_length));
    result.push_back(vert_l_x);
    result.push_back(vert_l_y);
    result.push_back(vert_l_x + (1.2 + inflat * 3.0) * cos(theta));
    result.push_back(vert_l_y + (1.2 + inflat * offset) * sin(theta));
    result.push_back(vert_l_x + hypot(1.2 + inflat * 3.0, 1.2 + inflat * offset) * cos(theta - atan2(1.2 + offset, 1.2 + 3.0)));
    result.push_back(vert_l_y + hypot(1.2 + inflat * 3.0, 1.2 + inflat * offset) * sin(theta - atan2(1.2 + offset, 1.2 + 3.0)));
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

class VVCM {
public:
  struct Point {
    double x;
    double y;
    Point() {}
    Point(double x_val, double y_val) : x(x_val), y(y_val) {}
  };
  double formation_radius = 1.5;
  double radius_inc = 0.1;
  double xv1 = 0.0;
  double yv1 = 0.0;
  double xv2 = 3.3;
  double yv2 = 0.0;
  double xv3 = xv2 / 2;
  double yv3 = sqrt(3) * xv3;
  double xv1t = 0.0;
  double yv1t = 0.0;
  double xv2t = 0.65;
  double yv2t = 0.0;
  double xv3t = xv2t / 2;
  double yv3t = sqrt(3) * xv3t;
  double zr = 2.2;
  double cal_distance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
  }
  double dotProduct(Point v1, Point v2) {
    return v1.x * v2.x + v1.y * v2.y;
  }
  void convertLocal(Point p1, Point p2, Point p3, Point& p1_convert, Point& p2_convert, Point& p3_convert) {
    Point p1p2(p2.x - p1.x, p2.y - p1.y);
    double length_p1p2 = cal_distance(p1, p2);
    Point unit_p1p2(p1p2.x / length_p1p2, p1p2.y / length_p1p2);
    Point p1p(p3.x - p1.x, p3.y - p1.y);
    double projection = dotProduct(unit_p1p2, p1p);
    double x_relative = projection;
    double y_relative = sqrt(pow(cal_distance(p1, p3), 2) - pow(projection, 2));
    p1_convert.x = 0.0;
    p1_convert.y= 0.0;
    p2_convert.x = length_p1p2;
    p2_convert.y = 0.0;
    p3_convert.x = x_relative;
    p3_convert.y = y_relative;
  }
  bool solveLinearEquation(double a1, double b1, double c1, double a2, double b2, double c2, double& x, double& y) {
    // 计算行列式的值
    double determinant = a1 * b2 - a2 * b1;

    // 如果行列式为0，方程无解
    if (determinant == 0) {
        return false;
    }

    // 计算解的值
    x = (c1 * b2 - c2 * b1) / determinant;
    y = (a1 * c2 - a2 * c1) / determinant;

    return true;
}
  void calvo(Point p1, Point p2, Point p3, Point& pvo, Point& ro, double& zo) {
    double a11 = (xv2 * xv2 - p2.x * p2.x) / xv2;
    double a12 = p2.x * (xv3 * p2.x - p3.x * xv2) / (xv2 * yv3);
    double b1 = (xv2 * xv2 - p2.x * p2.x) / 2;
    double a21 = xv3 * p2.x - p3.x * xv2;
    double a22 = p2.x * (yv3 * yv3 - p3.y * p3.y) / yv3;
    double b2 = p3.x * (p2.x * p2.x - xv2 * xv2)  / 2 + p2.x * (xv3 * xv3 + yv3 * yv3 - p3.x * p3.x - p3.y * p3.y) / 2;
    double xvo, yvo;
    if (solveLinearEquation(a11, a12, b1, a21, a22, b2, xvo, yvo)) {}
    pvo.x = xvo;
    pvo.y = yvo;
    ro.x = xv2 * xvo / p2.x + (p2.x * p2.x - xv2 * xv2) / (2 * p2.x);
    ro.y = (xv3 / p3.y - p3.x / p3.y * (xv2 / p2.x)) * xvo + yv3 * yvo / p3.y - (p3.x / p3.y) * (p2.x * p2.x - xv2 * xv2) / (2 * p2.x) + (p3.x * p3.x + p3.y * p3.y - xv3 * xv3 - yv3 * yv3) / (2 * p3.y);
    zo = fabs(zr - sqrt(xvo * xvo + yvo * yvo - ro.x * ro.x - ro.y  * ro.y));

  }
  void cak_direct_kinmatics(std::vector<double> pos_vec, std::vector<double>& result) {
    Point p1(pos_vec[0], pos_vec[1]);
    Point p2(pos_vec[2], pos_vec[3]);
    Point p3(pos_vec[4], pos_vec[5]);
    Point pvo, ro;
    double zo;
    Point p1_convert, p2_convert, p3_convert;
    convertLocal(p1, p2, p3, p1_convert, p2_convert, p3_convert);
    calvo(p1_convert, p2_convert, p3_convert, pvo, ro, zo);
    result.push_back(p2_convert.x);
    result.push_back(p3_convert.x);
    result.push_back(p3_convert.y);
    result.push_back(pvo.x);
    result.push_back(pvo.y);
    result.push_back(ro.x);
    result.push_back(ro.y);
    result.push_back(zo);
  }
};

}

#endif //SRC_VEHICLE_MODEL_H
