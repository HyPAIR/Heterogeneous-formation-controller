//
// Created by weijian on 17/11/23.
//

#ifndef SRC_ENVIRONMENT_H
#define SRC_ENVIRONMENT_H
#include <vector>
#include <costmap_2d/costmap_2d.h>

#include "math/polygon2d.h"
#include "planner_config.h"

namespace heterogeneous_formation_controller {

class Environment {
public:
  explicit Environment(std::shared_ptr<PlannerConfig> config): config_(std::move(config)) {}

  std::vector<math::Polygon2d> &polygons() { return polygons_; }
  std::vector<math::Vec2d> &points() { return points_; };
  std::vector<double> &heights() {return heights_; };

  struct EquationSystemResult {
    bool hasSolution;
    double x;
    double y;
  };

  bool CheckPoseCollision(double time, math::Pose pose) const;

  bool CheckVerticeCollision(double time, math::Pose pose) const;

  bool CheckHomotopyConstraints(double time, math::Pose pose, const std::vector<std::vector<double>> corridor_sets) const;

  bool CheckSpatialEnvelopes(double time, math::Pose pose) const;

  bool GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const;

  bool CheckBoxCollision(double time, const math::AABox2d &box) const;

  void UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap);

  std::vector<math::Vec2d> InflateObstacle(const std::vector<math::Vec2d>& vertices, double inflationRadius);

  EquationSystemResult SolveEquationSystem(double a1, double b1, double c1, double a2, double b2, double c2);

private:
  std::shared_ptr<PlannerConfig> config_;
  std::vector<math::Polygon2d> polygons_;
  std::vector<math::Vec2d> points_;
  std::vector<double> heights_;
};

}

#endif //SRC_ENVIRONMENT_H
