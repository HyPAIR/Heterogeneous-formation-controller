#include <iostream>
#include "heterogeneous_formation_controller/IdentifyHomotopy.h"
#include "heterogeneous_formation_controller/topo_prm.h"
#include "heterogeneous_formation_controller/visualization/plot.h"

namespace heterogeneous_formation_controller {
// Function to calculate the directed area
bool calculateSignedDistance(const Point& k, const Point& p, const Point& np) {
    Eigen::Matrix2d B;
    B << 0, -1, 
         1, 0;
    Eigen::Vector2d vec_npp(np.x - p.x, np.y - p.y);
    Eigen::Vector2d vec_kp(k.x - p.x, k.y - p.y);
    Eigen::Vector2d vec_left = B * vec_npp;
    double norm_left = vec_npp.norm();
    double result = vec_left.transpose().dot(vec_kp) / norm_left;
    return result > 0 ? true : false;
}

void generateBinaryCombinations(int n, int depth, std::vector<int>& current, std::vector<std::vector<int>>& result) {
    if (depth == n) {
        result.push_back(current);
        return;
    }

    // for each position, either 0 or 1
    for (int i = 0; i <= 1; ++i) {
        current.push_back(i); // select the current number
        generateBinaryCombinations(n, depth + 1, current, result); // retrive to fill the next position
        current.pop_back(); // back track, try the other number
    }
}

double calPathLength(vector<Eigen::Vector2d> raw_path) {
  double length = 0.0;
  for (int j = 1; j < raw_path.size(); j++) {
    length += hypot(raw_path[j].x() - raw_path[j - 1].x(), raw_path[j].y() - raw_path[j - 1].y());
  }
}
 
std::vector<int> evalPathLength(const vector<vector<Eigen::Vector2d>>& raw_paths) {
    std::vector<double> path_length;
    for (int i = 0; i < raw_paths.size(); i++) {
      path_length.push_back(calPathLength(raw_paths[i]));
    }
    auto index_set = findTwoSmallest(path_length);
    return index_set;
}

vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector2d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector2d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, std::shared_ptr<heterogeneous_formation_controller::Environment>& env) {
  // Eigen::Vector2d intersect_pt;
  Eigen::Vector2d q1, q2;
  for (int i = 0; i < env->polygons().size(); ++i) {
    if (lineIntersectsPolygon(p1, p2, env->polygons()[i], q1, q2)) {       
      return false;
    } 
  }
  return true;
}

bool JudgeHomotopy(vector<Pathset> paths_set, vector<int> combineation, std::shared_ptr<heterogeneous_formation_controller::Environment>& env) {
  for (int i = 0; i < combineation.size(); i++) {
    for (int j = 0; j < paths_set[0].path.size(); j++) {
      Eigen::Vector2d p1(paths_set[2 * (i + 1 ) % 3 + combineation[(i + 1) % 3]].path[j].x(), paths_set[2 * (i + 1 ) % 3 + combineation[(i + 1) % 3]].path[j].y());
      Eigen::Vector2d p2(paths_set[2 * i + combineation[i]].path[j].x(), paths_set[2 * i + combineation[i]].path[j].y());
      if (lineVisib(p1, p2, env)) return false;
    }
  }
  return true;
}

bool BeyondHeightCons(vector<Pathset> paths_set, vector<int> combineation, std::shared_ptr<heterogeneous_formation_controller::Environment>& env) {
  VVCM vvcm;
  for (int i = 0; i < combineation.size(); i++) {
    for (int j = 0; j < paths_set[0].path.size(); j++) {
      Eigen::Vector2d p1(paths_set[2 * (i + 1 ) % 6 + combineation[(i + 1) % 3]].path[j].x(), paths_set[2 * (i + 1 ) % 6 + combineation[(i + 1) % 3]].path[j].y());
      Eigen::Vector2d p2(paths_set[2 * i + combineation[i]].path[j].x(), paths_set[2 * i + combineation[i]].path[j].y());
      Eigen::Vector2d q1, q2;
      for (int i = 0; i < env->polygons().size(); ++i) {
        if (lineIntersectsPolygon(p1, p2, env->polygons()[i], q1, q2)) {       
          if (env->heights()[i] > vvcm.zr)
            return true;
        } 
      }
    }
  }
  return false;
}

std::vector<std::vector<std::vector<std::vector<double>>>> CalCorridors(const vector<vector<vector<Eigen::Vector2d>>>& raw_paths_set, 
    std::shared_ptr<heterogeneous_formation_controller::Environment>& env) {
    int num_selected_path = 2;
    vector<Pathset> paths_set;
    vector<double> path_length_set, path_length_set_new;
    std::vector<std::vector<int>> combinations, combinations_new;
    std::vector<int> current;
    vector<int> topolpgy_vio_set;
    std::vector<std::vector<int>> path_index_set(raw_paths_set.size());
    for (int ind = 0; ind < raw_paths_set.size(); ind++) {
        auto index_set = evalPathLength(raw_paths_set[ind]);
        for (int j = 0; j < num_selected_path; j++) {
          Pathset path{calPathLength(raw_paths_set[ind][index_set[j]]), discretizePath(raw_paths_set[ind][index_set[j]], 50)};
          paths_set.push_back(path);
        }
    }
    std::cout << "[Homotopy]: total num: " << paths_set.size();
    generateBinaryCombinations(raw_paths_set.size(), 0, current, combinations);
    for (auto combination : combinations) {
      int topolpgy_vio = 0;
      double path_length = 0.0;
      for (int i = 0; i < paths_set[0].path.size(); i++) {
      vector<Point> point_set;
        for (int j = 0; j < combination.size(); j++) {
          point_set.push_back(Point{paths_set[2 * j + combination[j]].path[i].x(), paths_set[2 * j + combination[j]].path[i].y()});
          path_length += calPathLength(paths_set[2 * j + combination[j]].path);
        }
        if (!calculateSignedDistance(point_set[0], point_set[1], point_set[2])) topolpgy_vio++;
      }
      topolpgy_vio_set.push_back(topolpgy_vio);
      path_length_set.push_back(path_length / 20);
    }
    // order by the length
    for (int i = 0; i < topolpgy_vio_set.size(); i++) {
      // if (topolpgy_vio_set[i] < 5) {
        combinations_new.push_back(combinations[i]);
        path_length_set_new.push_back(path_length_set[i]);
      // }
    }
    std::cout << ", pruned num: " << paths_set.size() - combinations_new.size();
    auto ordering_set = findTwoSmallest(path_length_set_new);
    combinations.clear();
    for (int i = 0; i < ordering_set.size(); i++) {
      if (!BeyondHeightCons(paths_set, combinations_new[ordering_set[i]], env)) {
        combinations.push_back(combinations_new[ordering_set[i]]);
      }
    }
    for (int i = 0; i < combinations.size(); i++) {
      if (JudgeHomotopy(paths_set, combinations[i], env)) {
        combinations.push_back(combinations[i]);
        combinations.erase(combinations.begin() + i);
      }
    }
    // odering by passing/crossing
    std::vector<std::vector<std::vector<std::vector<double>>>> corridors_sets(combinations.size());
    std::vector<std::vector<double>> corridor_sets(paths_set[0].path.size(), std::vector<double>(4, 0));
    std::vector<heterogeneous_formation_controller::math::Pose> topo_path;
    for (int ind = 0; ind < combinations.size(); ind++) {
      std::cout << ", combination: " << combinations[ind][0] << " ," << combinations[ind][1] << " ," << combinations[ind][2];
      for (int i = 0; i < combinations[0].size(); i++) {
        for (int j = 0; j < paths_set[0].path.size(); j++) {
            math::AABox2d box;
            if (!env->GenerateCorridorBox(0.0, paths_set[2 * i + combinations[ind][i]].path[j].x(), paths_set[2 * i + combinations[ind][i]].path[j].y(), 0.3, box)) {
                ROS_ERROR("corridor box indexed at %d on %dth path failed!", j, i);
            }
            corridor_sets[j][0] = box.min_x();
            corridor_sets[j][1] = box.min_y();
            corridor_sets[j][2] = box.max_x();
            corridor_sets[j][3] = box.max_y();
            // if (i == 2) {
            // auto color = visualization::Color::Magenta;
            // color.set_alpha(0.1);
            // visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.5, color, i, "Corridor " + std::to_string(j)); 
            // visualization::Trigger();
            // }
        }      
      corridors_sets[ind].push_back(corridor_sets);
      }
    }
     std::cout << "\n";
    return corridors_sets;
}

std::vector<int> findTwoSmallest(const std::vector<double>& vec) {
    // double min1 = std::numeric_limits<double>::max(); 
    // double min2 = std::numeric_limits<double>::max();
    // index1 = -1;
    // index2 = -1;

    // for (int i = 0; i < vec.size(); ++i) {
    //     if (vec[i] < min1) {
    //         min2 = min1;
    //         index2 = index1;
    //         min1 = vec[i];
    //         index1 = i;
    //     } else if (vec[i] < min2 && vec[i] != min1) {
    //         min2 = vec[i];
    //         index2 = i;
    //     }
    // }
    std::vector<int> indices(vec.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&vec](int i1, int i2) { return vec[i1] < vec[i2]; });

    return indices;
}

}