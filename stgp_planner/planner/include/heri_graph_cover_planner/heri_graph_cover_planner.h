
#ifndef ROBO_PLANNER_WS_HERI_GRAPH_COVER_PLANNER_H
#define ROBO_PLANNER_WS_HERI_GRAPH_COVER_PLANNER_H

#include "graph/plan_graph.h"
#include "preprocess/preprocess.h"
#include "tsp_solver/two_opt.h"
#include <ros/package.h>

#include <visualization_tools/IterationTime.h>
#include <visualization_tools/ViewpointGain.h>

namespace heri_graph_cover_planner {
using namespace utils;
using namespace perception;
using namespace preprocess;

typedef std::vector<utils::Point3D> Path;

class HeriGraphCoverPlanner {

public:
  typedef std::shared_ptr<HeriGraphCoverPlanner> Ptr;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  int max_tour_point_num_;
  double viewpoint_ignore_thre_;
  double local_range_;
  double frontier_gain_;
  double tourpoint_ignore_distance_;
  double tourpoint_ignore_thre_;

  perception::Ufomap::Ptr ufo_map_;
  preprocess::GlobalTopoManager::Ptr global_topo_map_;

  graph::PlanGraph plan_graph_;
  utils::Point3DSet tour_points_;
  utils::Point3DMap<double> tour_points_gains_;
  double max_gain_;
  std::vector<std::vector<Path>> path_matrix_;
  utils::Point3DMap<utils::Point3DMap<Path>> pre_paths_;

  int current_explore_branch_;
  utils::NodeQueue tour_topo_path_;
  utils::NodeQueue passed_junctions_;
  NodeQueue history_local_frontiers_;
  std::vector<MissedBranchMap> history_missed_branches_;
  utils::NodeSet history_unconfirmed_labels_;
  Node last_target_node_ = -1;
  bool is_frontier_guiding_ = false;
  bool is_frontier_robot_same_ = false;
  bool need_ufo_frontier_ = false;
  bool no_frontier_left_ = false;
  bool during_relocate_ = false;
  CodeUnorderSet local_frontier_cells_copy_;

  utils::Point3D goal_point_;
  Path path_to_go_;
  Path tsp_path_;
  std::vector<Path> path_segments_;

  FrontierQueue goal_point_frontiers_;

  bool is_local_planning_;

  bool is_directory_;
  double alpha_;

  std::string each_solving_txt_name_;
  double sum_solving_time_;
  int solving_num_;

  std::string each_tourpoints_initilize_txt_name_;
  double sum_initilize_time_;
  int initilize_num_;

  std::mutex planner_mutex_;

  std::string two_opt_time_name_;
  double sum_two_opt_time;

  std::vector<double> frontiers_gains_;
  std::vector<double> unmapped_gains_;
  std::vector<double> mean_errors_;

  HeriGraphCoverPlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                        const Ufomap::Ptr &ufo_map,
                        const GlobalTopoManager::Ptr &global_topo_manager);

  void setParamsFromRos();

  void Initialize(const Point3D &current_position);

  void planGraphConstruct(const graph::PlanGraph &old_graph,
                          graph::PlanGraph &new_graph);

  void getSuitableTourPoints(const Point3D &current_position);

  void viewpointsFrontierGain(utils::Point3DSet &viewpoints,
                              utils::Point3DMap<double> &tour_points_gains,
                              double &max_gain);

  void planning(const geometry_msgs::Pose &current_pose,
                const utils::Point3D &current_directory, bool &is_successed);

  bool heri_topo_planning(const geometry_msgs::Pose &current_pose);

  bool two_opt_solve_planning(const geometry_msgs::Pose &current_pose,
                              const utils::Point3D &current_directory);

  int addCurrentPositionToGraph(const Point3D &current_position,
                                graph::PlanGraph &graph);

  Path getPathInGraph(const int &start_point_id, const int &end_point_id,
                      const graph::PlanGraph &graph);

  Path getPathInGridMap2D(const Point3D &start_point, const Point3D &end_point,
                          const GridMap2D &grid_map_2d);

  Path MYgetPathInGridMap2D(const Point3D &start_point,
                            const Point3D &end_point,
                            const GridMap2D &grid_map_2d,
                            const GlobalTopoManager::Ptr &topo_manager,
                            int &approximate_Manhattan_dis);

  static double getPathLength(const Path &path);
};

}

#endif
