//
// Created by weijian on 17/11/23.
//

#include "heterogeneous_formation_controller/environment.h"
#include <costmap_2d/cost_values.h>

namespace heterogeneous_formation_controller {

bool Environment::CheckBoxCollision(double time, const math::AABox2d &box) const {
  // TODO: reimplement using R-Tree
  for(auto &polygon: polygons_) {
    if(polygon.HasOverlap(math::Box2d(box))) {
      return true;
    }
  }

  for(auto &point: points_) {
    if(box.IsPointIn(point)) {
       return true;
    }
  }

  return false;
}

bool Environment::CheckPoseCollision(double time, math::Pose pose) const {
  auto discs = config_->vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());
  double wh = config_->vehicle.disc_radius * 2;
  for(int i = 0; i < discs.size() / 2; i++) {
    if(CheckBoxCollision(time, math::AABox2d({discs[i * 2], discs[i * 2 + 1]}, wh, wh))) {
      return true;
    }
  }

  return false;
}

bool Environment::CheckVerticeCollision(double time, math::Pose pose) const {
  auto f_centre = config_->vehicle.GetFormationCentre(pose.x(), pose.y(), pose.theta());
  if(CheckBoxCollision(time, math::AABox2d({f_centre[0], f_centre[1]}, 2.0 + 1.2, config_->vehicle.offset + 1.2))) {
    return true;
  }
  return false;
}


bool Environment::CheckHomotopyConstraints(double time, math::Pose pose, const std::vector<std::vector<double>> corridor_sets) const {
  for (int i = 0; i < corridor_sets.size(); i++) {
    if (pose.x() >= corridor_sets[i][0] && pose.x() <= corridor_sets[i][2]
        && pose.y() >= corridor_sets[i][1] && pose.y() <= corridor_sets[i][3]) {
          return true;
        }
  }
  return false;
}

bool Environment::CheckSpatialEnvelopes(double time, math::Pose pose) const {
  auto f_centre = config_->vehicle.GetFormationCentre(pose.x(), pose.y(), pose.theta());
  if(CheckBoxCollision(time, math::AABox2d({f_centre[0], f_centre[1]}, sqrt(2) * (2.0 + 1.2), sqrt(2) * (config_->vehicle.offset + 1.2)))) {
    return true;
  }
  return false;
}

void Environment::UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap) {
  points_.clear();
  // 遍历每一个网格
  for(int i = 0; i < costmap->getSizeInCellsX()-1; i++) {
    for(int j = 0; j < costmap->getSizeInCellsY()-1; j++) {
      if(costmap->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE) {
        double obs_x, obs_y;
        costmap->mapToWorld(i,j, obs_x, obs_y);
        points_.emplace_back(obs_x, obs_y);
      }
    }
  }
}

bool Environment::GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const {
  double ri = radius;
  // 或许应该是math::AABox2d bound({x-ri, y-ri}, {x+ri, y+ri});
  math::AABox2d bound({-ri, -ri}, {ri, ri});

  if(CheckBoxCollision(time, bound)) {
    // initial condition not satisfied, involute to find feasible box
    int inc = 4;
    double real_x, real_y;

    do {
      int iter = inc / 4;
      uint8_t edge = inc % 4;

      real_x = x;
      real_y = y;
      if(edge == 0) {
        real_x = x - iter * 0.05;
      } else if(edge == 1) {
        real_x = x + iter * 0.05;
      } else if(edge == 2) {
        real_y = y - iter * 0.05;
      } else if(edge == 3) {
        real_y = y + iter * 0.05;
      }

      inc++;
      bound = math::AABox2d({real_x-ri, real_y-ri}, {real_x+ri, real_y+ri});
    } while(CheckBoxCollision(time, bound) && inc < config_->corridor_max_iter);
    if(inc > config_->corridor_max_iter) {
      return false;
    }

    x = real_x;
    y = real_y;
  }

  int inc = 4;
  std::bitset<4> blocked;
  double incremental[4] = {0.0};
  double step = radius * 0.2;

  do {
    int iter = inc / 4;
    uint8_t edge = inc % 4;
    inc++;

    if (blocked[edge]) continue;

    incremental[edge] = iter * step;

    math::AABox2d test({-ri - incremental[0], -ri - incremental[2]},
                 {ri + incremental[1], ri + incremental[3]});

    if (CheckBoxCollision(time, test.Offset({x, y})) || incremental[edge] >= config_->corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }
  } while (!blocked.all() && inc < config_->corridor_max_iter);
  if (inc > config_->corridor_max_iter) {
    return false;
  }

  result = {{x - incremental[0], y - incremental[2]},
            {x + incremental[1], y + incremental[3]}};
  return true;
}

std::vector<math::Vec2d> Environment::InflateObstacle(const std::vector<math::Vec2d>& vertices, double inflationRadius) {
    std::vector<math::Vec2d> inflatedObstacle;
    std::vector<math::Vec2d> inflatedPolygon;

    // Iterate over each vertex of the polygon
    for (size_t i = 0; i < vertices.size(); i++) {
        // Get the current, previous, and next vertices
        math::Vec2d currentVertex = vertices[i];
        math::Vec2d prevVertex = vertices[(i + vertices.size() - 1) % vertices.size()];
        math::Vec2d nextVertex = vertices[(i + 1) % vertices.size()];

        // Calculate the direction vectors of the adjacent edges
        double prevEdgeX = currentVertex.x() - prevVertex.x();
        double prevEdgeY = currentVertex.y() - prevVertex.y();
        double nextEdgeX = nextVertex.x() - currentVertex.x();
        double nextEdgeY = nextVertex.y() - currentVertex.y();

        // Normalize the direction vectors
        double prevEdgeLength = std::sqrt(prevEdgeX * prevEdgeX + prevEdgeY * prevEdgeY);
        double nextEdgeLength = std::sqrt(nextEdgeX * nextEdgeX + nextEdgeY * nextEdgeY);
        prevEdgeX /= prevEdgeLength;
        prevEdgeY /= prevEdgeLength;
        nextEdgeX /= nextEdgeLength;
        nextEdgeY /= nextEdgeLength;

        // Calculate the perpendicular vectors of the adjacent edges
        double prevPerpendicularX = -prevEdgeY;
        double prevPerpendicularY = prevEdgeX;
        double nextPerpendicularX = -nextEdgeY;
        double nextPerpendicularY = nextEdgeX;

        // Calculate the inflated vertices
        double prevX = currentVertex.x() + inflationRadius * (prevPerpendicularX);
        double prevY = currentVertex.y() + inflationRadius * (prevPerpendicularY);
        double nextX = currentVertex.x() + inflationRadius * (nextPerpendicularX);
        double nextY = currentVertex.y() + inflationRadius * (nextPerpendicularY);
        // Define the coefficients of the equation system
        double a1 = prevX - currentVertex.x();
        double b1 = prevY - currentVertex.y();
        double c1 = prevX * (prevX - currentVertex.x()) + prevY * (prevY - currentVertex.y());
        double a2 = nextX - currentVertex.x();
        double b2 = nextY - currentVertex.y();
        double c2 = nextX * (nextX - currentVertex.x()) + nextY * (nextY - currentVertex.y());
        // Solve the equation system
        EquationSystemResult result = SolveEquationSystem(a1, b1, c1, a2, b2, c2);

        // Add the inflated vertex to the inflated polygon
        inflatedPolygon.push_back({result.x, result.y});
    }

    return inflatedPolygon;
}

Environment::EquationSystemResult Environment::SolveEquationSystem(double a1, double b1, double c1, double a2, double b2, double c2) {
    double determinant = a1 * b2 - a2 * b1;

    Environment::EquationSystemResult result;
    result.hasSolution = false;

    if (determinant != 0) {
        result.hasSolution = true;
        result.x = (c1 * b2 - c2 * b1) / determinant;
        result.y = (a1 * c2 - a2 * c1) / determinant;
    }

    return result;
}

}