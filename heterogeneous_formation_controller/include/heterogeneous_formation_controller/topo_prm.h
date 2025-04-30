/**
 * file topo_prm.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief topological PRM
 * data 2024-4-2
 * 
 * @copyright Copyroght(c) 2024
*/

#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H

#include <random>
#include <iostream>
#include <Eigen/Eigen>
#include <list>
#include <vector>
// #include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>
#include <memory>
#include "heterogeneous_formation_controller/environment.h"
#include "heterogeneous_formation_controller/math/pose.h"
#include "heterogeneous_formation_controller/planner_config.h"

using std::vector;
using std::list;
using std::max;
using std::shared_ptr;
using std::random_device;
using std::default_random_engine;
using std::uniform_real_distribution;

namespace heterogeneous_formation_controller {

/* ---------- used for iterating all topo combination ---------- */
class TopoIterator {
private:
  /* data */
  vector<int> path_nums_;
  vector<int> cur_index_;
  int combine_num_;
  int cur_num_;

  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  TopoIterator(vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    std::cout << "[Topo]: merged path num: " << combine_num_ << std::endl;
  }
  TopoIterator() {
  }
  ~TopoIterator() {
  }

  bool nextIndex(vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
class GraphNode {
private:
  /* data */

public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode(/* args */) {
  }
  GraphNode(Eigen::Vector2d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {
  }

  typedef shared_ptr<GraphNode> Ptr;
  vector<Ptr> neighbors_;
  Eigen::Vector2d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

};

class TopologyPRM {
private:
  /* data */
  // sampling generator
  random_device rd_;
  default_random_engine eng_;
  uniform_real_distribution<double> rand_pos_;

  Eigen::Vector2d sample_r_;
  Eigen::Vector2d translation_;
  Eigen::Matrix2d rotation_;

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  // list作为双向链表，不支持像vector通过索引进行查询元素（需要从头部或尾部进行遍历），但能够快速插入和删除元素
  list<GraphNode::Ptr> graph_;
  vector<vector<Eigen::Vector2d>> raw_paths_;
  vector<vector<Eigen::Vector2d>> short_paths_;
  vector<vector<Eigen::Vector2d>> final_paths_;
  // vector<Eigen::Vector2d> start_pts_, end_pts_;

  // raycasting
  Eigen::Vector2d offset_;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_, max_raw_path2_;
  int short_cut_num_;
  Eigen::Vector2d sample_inflate_;
  double resolution_;

  double ratio_to_short_;
  int reserve_num_;

  bool parallel_shortcut_;

  shared_ptr<PlannerConfig> config_;
  shared_ptr<Environment> env_;

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  // create the graph
  list<GraphNode::Ptr> createGraph(Eigen::Vector2d start, Eigen::Vector2d end);
  vector<vector<Eigen::Vector2d>> searchPaths();
  void shortcutPaths();
  vector<vector<Eigen::Vector2d>> pruneEquivalent(vector<vector<Eigen::Vector2d>>& paths);
  vector<vector<Eigen::Vector2d>> selectShortPaths(vector<vector<Eigen::Vector2d>>& paths, int step);

  /* ---------- helper ---------- */
  inline Eigen::Vector2d getSample();
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector2d pt);  // find pairs of visibile guard
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                      Eigen::Vector2d pt);  // test redundancy with existing
                                            // connection between two guard
  bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, Eigen::Vector2d& pc);
  bool triangleVisib(Eigen::Vector2d pt, Eigen::Vector2d p1, Eigen::Vector2d p2);
  void pruneGraph();

  void depthFirstSearch(vector<GraphNode::Ptr>& vis);

  vector<Eigen::Vector2d> discretizeLine(Eigen::Vector2d p1, Eigen::Vector2d p2);
  vector<vector<Eigen::Vector2d>> discretizePaths(vector<vector<Eigen::Vector2d>>& path);

  vector<Eigen::Vector2d> discretizePath(vector<Eigen::Vector2d> path);
  void shortcutPath(vector<Eigen::Vector2d> path, int path_id, int iter_num = 1);

  vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path, int pt_num);
  bool sameTopoPath(const vector<Eigen::Vector2d>& path1, const vector<Eigen::Vector2d>& path2,
                    double thresh);
  Eigen::Vector2d getOrthoPoint(const vector<Eigen::Vector2d>& path);

  int shortestPath(vector<vector<Eigen::Vector2d>>& paths);

public:
  double clearance_;

  TopologyPRM(
      std::shared_ptr<PlannerConfig> config,
      std::shared_ptr<Environment> env): config_(std::move(config)), env_(std::move(env)
  ){}
  // ~TopologyPRM();

  void init(ros::NodeHandle& nh);

  void findTopoPaths(Eigen::Vector2d start, Eigen::Vector2d end, list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector2d>>& raw_paths,
                     vector<vector<Eigen::Vector2d>>& filtered_paths,
                     vector<vector<Eigen::Vector2d>>& select_paths);

  double pathLength(const vector<Eigen::Vector2d>& path);
  vector<Eigen::Vector2d> pathToGuidePts(vector<Eigen::Vector2d>& path, int pt_num);

};

}  // namespace fast_planner

#endif