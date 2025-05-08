
#include "heri_graph_cover_planner/heri_graph_cover_planner.h"

namespace heri_graph_cover_planner {

HeriGraphCoverPlanner::HeriGraphCoverPlanner(
    ros::NodeHandle &nh, ros::NodeHandle &nh_private,
    const Ufomap::Ptr &ufo_map,
    const GlobalTopoManager::Ptr &global_topo_manager)
    : nh_(nh), nh_private_(nh_private) {
  ufo_map_ = ufo_map;
  global_topo_map_ = global_topo_manager;

  setParamsFromRos();
}

void HeriGraphCoverPlanner::setParamsFromRos() {
  std::string ns = ros::this_node::getName() + "/HeriGraphCoverPlanner";

  std::string pkg_path = ros::package::getPath("planner");
  std::string txt_path = pkg_path + "/../../files/exploration_data/";

  max_tour_point_num_ = 10;
  if (!ros::param::get(ns + "/max_tour_point_num", max_tour_point_num_)) {
    ROS_WARN("No max_tour_point_num specified. Looking for %s. Default is 10",
             (ns + "/max_tour_point_num").c_str());
  }

  viewpoint_ignore_thre_ = 1.0;
  if (!ros::param::get(ns + "/viewpoint_ignore_thre", viewpoint_ignore_thre_)) {
    ROS_WARN(
        "No viewpoint_ignore_thre specified. Looking for %s. Default is 1.0",
        (ns + "/viewpoint_ignore_thre").c_str());
  }

  tourpoint_ignore_distance_ = 3.0;
  if (!ros::param::get(ns + "/tourpoint_ignore_distance",
                       tourpoint_ignore_distance_)) {
    ROS_WARN("No tourpoint_ignore_distance specified. Looking for %s. Default "
             "is 3.0",
             (ns + "/tourpoint_ignore_distance").c_str());
  }

  tourpoint_ignore_thre_ = 2.0;
  if (!ros::param::get(ns + "/tourpoint_ignore_thre", tourpoint_ignore_thre_)) {
    ROS_WARN(
        "No tourpoint_ignore_thre specified. Looking for %s. Default is 2.0",
        (ns + "/tourpoint_ignore_thre").c_str());
  }

  local_range_ = 10.0;
  if (!ros::param::get(ns + "/local_range", local_range_)) {
    ROS_WARN("No local_range specified. Looking for %s. Default is 1.0",
             (ns + "/local_range").c_str());
  }
  frontier_gain_ = 1.0;
  if (!ros::param::get(ns + "/frontier_gain", frontier_gain_)) {
    ROS_WARN("No frontier_gain specified. Looking for %s. Default is 1.0",
             (ns + "/frontier_gain").c_str());
  }

  is_local_planning_ = true;

  is_directory_ = true;
  if (!ros::param::get(ns + "/is_directory", is_directory_)) {
    ROS_WARN("No is_directory specified. Looking for %s. Default is true",
             (ns + "/is_directory").c_str());
  }

  alpha_ = 0.5;
  if (!ros::param::get(ns + "/alpha", alpha_)) {
    ROS_WARN("No alpha specified. Looking for %s. Default is 0.5",
             (ns + "/alpha").c_str());
  }

  each_tourpoints_initilize_txt_name_ =
      txt_path + "each_tourpoints_initilize_time.txt";
  each_solving_txt_name_ = txt_path + "each_solving_time.txt";

  std::ofstream fout;

  sum_initilize_time_ = 0;
  initilize_num_ = 0;
  fout.open(each_tourpoints_initilize_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
  fout << "each detection elisped time \n"
       << "start time \t"
       << "end time \t"
       << "elisped time \t"
       << "detection_num \t"
       << "average time \t" << std::endl;
  fout.close();

  sum_solving_time_ = 0;
  solving_num_ = 0;
  fout.open(each_solving_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
  fout << "each iteration elisped time \n"
       << "iteration start time \t"
       << " iteration end time \t"
       << " iteration elisped time \t"
       << "iteration_num \t"
       << "average time \t" << std::endl;
  fout.close();

  two_opt_time_name_ = txt_path + "two_opt_time.txt";
  fout.open(two_opt_time_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
  fout << "solve_number\t"
       << "\t"
       << "solve_time(s)\t"
       << "mean_time(s)\t"
       << "points_num\t" << std::endl;
  fout.close();
  sum_two_opt_time = 0.0;
}

void HeriGraphCoverPlanner::planGraphConstruct(
    const graph::PlanGraph &old_graph, graph::PlanGraph &new_graph) {
  new_graph.clearGraph();
  new_graph = old_graph;
}

void HeriGraphCoverPlanner::Initialize(const Point3D &current_position) {
  ROS_INFO("start initializing ....");
  ros::WallTime start_time = ros::WallTime::now();
  initilize_num_++;


  if (initilize_num_ == 1) {
    current_explore_branch_ = global_topo_map_->home_node_label_;
  }


  if (!during_relocate_) {
    tour_topo_path_.clear();
    bool exist_new_missed_branches;
    bool exist_frontier_in_current_explore_branch =
        global_topo_map_->frontierClassify(current_explore_branch_,
                                           history_missed_branches_,
                                           exist_new_missed_branches);
    if (exist_frontier_in_current_explore_branch) {
      if (false)
      {
        tour_topo_path_.push_back(
            global_topo_map_->local_frontier_labels_.front());
        during_relocate_ = false;
      }
      else
      {
        Node target_label = global_topo_map_->extendUnconfirmedFrontier(
          current_explore_branch_, global_topo_map_->local_frontier_labels_.front(), 10
        );
        tour_topo_path_.push_back(target_label);
        during_relocate_ = false;
      }

      if (tour_topo_path_.back() == last_target_node_) {
        if (!is_frontier_guiding_) {
          if (is_frontier_robot_same_)
            is_frontier_guiding_ = true;
        }
      }
      else {
        last_target_node_ = tour_topo_path_.back();
        is_frontier_robot_same_ = false;
        is_frontier_guiding_ = false;
      }

      if (is_frontier_guiding_) {
        if (false)
        {
        }
        else
        {
          ROS_INFO("Start ufomap frontier guiding");
          getSuitableTourPoints(current_position);
          tour_points_gains_.clear();
          max_gain_ = 0.0;
        }
      }
    }

    else if (exist_new_missed_branches) {
      ROS_INFO("current_explore_branch_ changing...");
      global_topo_map_->updateCurrentExploreBranch(
          current_explore_branch_, history_missed_branches_, passed_junctions_);
      ROS_INFO("updateCurrentExploreBranch func finished");
      tour_topo_path_.push_back(
          global_topo_map_->local_frontier_labels_.front());
      during_relocate_ = false;
      is_frontier_guiding_ = false;
      ROS_INFO("current_explore_branch_ changed");
    }

    else {
      ROS_INFO("Relocating... ");
      assert(global_topo_map_->local_frontier_labels_.empty());
      bool find_reloacte_branch;
      find_reloacte_branch = global_topo_map_->localFrontierRelocate(
          current_explore_branch_, history_missed_branches_, tour_topo_path_);
      if (find_reloacte_branch) {
        assert(!tour_topo_path_.empty());
        during_relocate_ = true;
      }
      else {
        ROS_INFO("There is no missed branch for explore may finished");
        during_relocate_ = false;
      }
      is_frontier_guiding_ = false;
    }
  }

  ros::WallTime end_time = ros::WallTime::now();
  ROS_INFO("fast ray casting gain computed finish,spent %f s",
           (end_time - start_time).toSec());
  sum_initilize_time_ = sum_initilize_time_ + (end_time - start_time).toSec();
  std::ofstream fout;
  fout.open(each_tourpoints_initilize_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::app);
  fout << start_time << "\t" << end_time << "\t"
       << (end_time - start_time).toSec() << "\t" << initilize_num_ << "\t"
       << sum_initilize_time_ / initilize_num_ << "s \t" << std::endl;
  fout.close();
}

void HeriGraphCoverPlanner::getSuitableTourPoints(
    const Point3D &current_position) {

  tour_points_.clear();

  utils::Point3DSet local_viewpoints;
  utils::Point3DSet global_viewpoints;


  double min_distancd = 1000000.0;
  ufo::map::Point3 nearest_point;
  for (const auto &code : global_topo_map_->ufomap_->local_frontier_cells_) {
    ufo::map::Point3 point = global_topo_map_->ufomap_->map_.toCoord(code.toKey());
    double distance = ((point.x()-current_position.x()) * (point.x()-current_position.x())) + ((point.y()-current_position.y()) * (point.y()-current_position.y()));
    if (distance < min_distancd) {
      min_distancd = distance;
      nearest_point = point;
    }
  }
  if (min_distancd < 1000000.0) {
    local_viewpoints.insert(Point3D(
      (nearest_point.x()+current_position.x())*0.5, 
      (nearest_point.y()+current_position.y())*0.5, 
      nearest_point.z()));
  }


  if (local_viewpoints.empty()) {
    ROS_INFO("GET frontier from global frontiers");
    double min_distancd = 1000000.0;
    ufo::map::Point3 nearest_point;
    for (const auto &code : global_topo_map_->ufomap_->global_frontier_cells_) {
      ufo::map::Point3 point = global_topo_map_->ufomap_->map_.toCoord(code.toKey());
        double distance = ((point.x()-current_position.x()) * (point.x()-current_position.x())) + ((point.y()-current_position.y()) * (point.y()-current_position.y()));
        if (distance < min_distancd) {
          min_distancd = distance;
          nearest_point = point;
        }
    }
    if (min_distancd < 1000000.0) {
      global_viewpoints.insert(Point3D(
        (nearest_point.x()+current_position.x())*0.5, 
        (nearest_point.y()+current_position.y())*0.5, 
        nearest_point.z()));
    }
  }


  if (!local_viewpoints.empty()) {
    is_local_planning_ = true;
    tour_points_ = local_viewpoints;
  } else {
    is_local_planning_ = false;
    tour_points_ = global_viewpoints;
  }

  ROS_INFO(" tour points detection finished, get %zu points to view",
           tour_points_.size());
}

void HeriGraphCoverPlanner::viewpointsFrontierGain(
    utils::Point3DSet &viewpoints, utils::Point3DMap<double> &tour_points_gains,
    double &max_gain) {
  ROS_INFO("start tour points gain computing...");

  for (const auto &viewpoint : viewpoints) {
    FrontierQueue viewpoint_visual_frontiers;
    tour_points_gains[viewpoint] = 1.0;
    if (tour_points_gains[viewpoint] > max_gain) {
      max_gain = tour_points_gains[viewpoint];
    }
  }
}

void HeriGraphCoverPlanner::planning(const geometry_msgs::Pose &current_pose,
                                     const utils::Point3D &current_directory,
                                     bool &is_successed) {
  planner_mutex_.lock();
  solving_num_++;

  auto start_time = std::chrono::high_resolution_clock::now();
  double start_time_second =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              start_time.time_since_epoch())
                              .count()) /
      1000000;

  if (is_frontier_guiding_) {
    if (two_opt_solve_planning(current_pose, current_directory)) {
      ROS_INFO("this iteration planning successed.");
      is_successed = true;
    } else {
      ROS_WARN("this iteration planning failed");
      is_successed = false;
    }
  }
  else {
    assert(!no_frontier_left_);
    if (tour_topo_path_.empty()) {
      ROS_INFO("tour_topo_path_ is empty");
      is_successed = false;
    }
    else {
      if (heri_topo_planning(current_pose)) {
        ROS_INFO("this iteration planning successed.");
        is_successed = true;
      } else {
        ROS_WARN("this iteration planning failed");
        is_successed = false;
      }
    }
  }

  auto finish_time = std::chrono::high_resolution_clock::now();
  double finish_time_second =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              finish_time.time_since_epoch())
                              .count()) /
      1000000;

  double iteration_time = finish_time_second - start_time_second;
  sum_solving_time_ = sum_solving_time_ + iteration_time;
  std::ofstream time_fout;
  std::ofstream count_fout;
  time_fout.precision(16);
  count_fout.precision(6);

  time_fout.open(each_solving_txt_name_,
                 std::ios_base::in | std::ios_base::out | std::ios_base::app);
  time_fout << start_time_second << "\t" << finish_time_second << "\t";
  time_fout.close();
  count_fout.open(each_solving_txt_name_,
                  std::ios_base::in | std::ios_base::out | std::ios_base::app);
  count_fout << iteration_time << "\t" << solving_num_ << "\t"
             << sum_solving_time_ / solving_num_ << "s \t"
             << tour_points_.size() << std::endl;
  count_fout.close();

  planner_mutex_.unlock();
}

bool HeriGraphCoverPlanner::heri_topo_planning(
    const geometry_msgs::Pose &current_pose) {
  path_to_go_.clear();
  tsp_path_.clear();
  path_segments_.clear();
  Point3D current_position(current_pose.position.x, current_pose.position.y,
                           current_pose.position.z);
  int path_size = tour_topo_path_.size();
  if (path_size == 1) {
    Point2D position =
        global_topo_map_->index2Point(global_topo_map_->nodeLabel2Index(tour_topo_path_.back()));
    int approx_dis;
    tsp_path_ = MYgetPathInGridMap2D(
        current_position,
        Point3D(position.x(), position.y(), current_pose.position.z),
        global_topo_map_->grid_map_, global_topo_map_, approx_dis);
    path_segments_.push_back(tsp_path_);
    if (tsp_path_.empty()) {
      return false;
    }
    else {
      if (tsp_path_.size()<2) {
        ROS_INFO("current and goal is the same point");
        is_frontier_robot_same_ = true;
      }
      if (during_relocate_) {
        if (tsp_path_.size() < 20) {
          assert(global_topo_map_->local_frontier_labels_.empty());
          global_topo_map_->updateLocalFrontier(current_explore_branch_);
          during_relocate_ = false;
        }
      }
      return true;
    }
  }

  assert(path_size > 1);
  utils::Point3DQueue point3d_path;
  for (Node &node : tour_topo_path_) {
    Point2D position =
        global_topo_map_->index2Point(global_topo_map_->nodeLabel2Index(node));
    point3d_path.emplace_back(position.x(), position.y(),
                              current_pose.position.z);
  }

  Path path_matrix[path_size - 1];
  std::vector<int> approx_path_length(path_size - 1);
  int approximate_Manhattan_dis;
  for (int i = 0; i < path_size - 1; ++i) {
    path_matrix[i] = MYgetPathInGridMap2D(point3d_path[i], point3d_path[i + 1],
                                        global_topo_map_->grid_map_, global_topo_map_,
                                        approximate_Manhattan_dis);
    approx_path_length[i] = approximate_Manhattan_dis;
    if (path_matrix[i].empty()) {
      ROS_INFO("can not find the path bwtween tour point "
                "(%f，%f)",
                point3d_path[i].x(), point3d_path[i].y());
      return false;
    }
  }

  Path path_matrix_from_current[path_size];
  int approx_path_length_from_current_to_last_i;
  path_matrix_from_current[0] = MYgetPathInGridMap2D(current_position,
    point3d_path[0], global_topo_map_->grid_map_, global_topo_map_, approx_path_length_from_current_to_last_i);
  if (path_matrix_from_current[0].empty()) {
    ROS_INFO("can not find the path from current position to tour point ");
    return false;
  }
  int valid_index=0;
  if (approx_path_length_from_current_to_last_i > 10)
    valid_index = 0;
  else {
    for (int i = 1; i < path_size; ++i) {
      int approx_path_length_from_current_to_i;
      path_matrix_from_current[i] = MYgetPathInGridMap2D(current_position,
        point3d_path[i], global_topo_map_->grid_map_, global_topo_map_, approx_path_length_from_current_to_i);
      if (path_matrix[i].empty()) {
        ROS_ERROR("can not find the path when finding valid tour points"
                  "(%f，%f)",
                  point3d_path[i].x(), point3d_path[i].y());
        return false;
      }
      int approx_path_length_between_ii = approx_path_length[i-1];
      if (approx_path_length_between_ii + approx_path_length_from_current_to_last_i >= approx_path_length_from_current_to_i - 1) {
        approx_path_length_from_current_to_last_i = approx_path_length_from_current_to_i;
      } else {
        valid_index = i - 1;
        break;
      }
      valid_index = i;
    }
  }

  tsp_path_.insert(tsp_path_.end(),
                   path_matrix_from_current[valid_index].begin(),
                   path_matrix_from_current[valid_index].end());
  path_segments_.push_back(path_matrix_from_current[valid_index]);
  utils::NodeQueue tour_topo_path_tmp;
  for (int j = valid_index; j < path_size-1; ++j) {
    tour_topo_path_tmp.push_back(tour_topo_path_[j]);
    tsp_path_.insert(tsp_path_.end(),
                     path_matrix[j].begin(),
                     path_matrix[j].end());
    path_segments_.push_back(path_matrix[j]);
  }
  tour_topo_path_ = tour_topo_path_tmp;

  if (during_relocate_) {
    if (path_segments_.size() < 3) {
      assert(global_topo_map_->local_frontier_labels_.empty());
      global_topo_map_->updateLocalFrontier(current_explore_branch_);
      during_relocate_ = false;
    }
  }

  return true;
}

  bool HeriGraphCoverPlanner::two_opt_solve_planning(
      const geometry_msgs::Pose &current_pose,
      const utils::Point3D &current_directory) {
    Point3D current_position(current_pose.position.x, current_pose.position.y,
                             current_pose.position.z);

    path_to_go_.clear();
    tsp_path_.clear();
    path_segments_.clear();
    if (tour_points_.empty()) {
      if (plan_graph_.getAllVertices().size() < 5) {
        ROS_INFO("plan_map is not construct or vertex num is too few, please "
                 "wait a minute...");
      } else
        ROS_WARN(" the candidate points is empty, exploration finish. ");
      return false;
    } else {

      Point3DQueue tour_points_term;
      tour_points_term.push_back(current_position);
      for (const auto &point : tour_points_) {
        tour_points_term.push_back(point);
      }
      int tour_points_num = tour_points_term.size();

      ROS_INFO("start get path maxtirx of tour points");

      Path path_matrix[tour_points_num][tour_points_num];
      Path empty_path;
      for (int i = 0; i < tour_points_num; ++i) {
        for (int j = 0; j < tour_points_num; ++j) {
          path_matrix[i][j] = empty_path;
        }
      }
      for (int i = 1; i < tour_points_num; i++) {
        ROS_INFO("local point get path in terrain map..");
        Path path = getPathInGridMap2D(current_position, tour_points_term.at(i),
                                       global_topo_map_->grid_map_);
        path_matrix[0][i] = path;
        std::reverse(path.begin(), path.end());
        path_matrix[i][0] = path;
        if (path.empty()) {
          ROS_ERROR("can not find the path from current position to tour point "
                    "(%f，%f)",
                    tour_points_term.at(i).x(), tour_points_term.at(i).y());
        }
      }
      ROS_INFO("start get other tour points paths..");
      for (int i = 1; i < tour_points_num; ++i) {
        for (int j = 1; j < tour_points_num; ++j) {
          if (i == j) {
            path_matrix[i][j] = empty_path;
          } else if (path_matrix[i][j].empty()) {
            Path path = getPathInGridMap2D(tour_points_term.at(i),
                                           tour_points_term.at(j),
                                           global_topo_map_->grid_map_);
            if (path.size() >= 2) {
            }
            if (path.empty()) {
              ROS_ERROR(
                  "can not find the path from point (%f，%f) to point (%f，%f)",
                  tour_points_term.at(i).x(), tour_points_term.at(i).y(),
                  tour_points_term.at(j).x(), tour_points_term.at(j).y());
            }
            path_matrix[i][j] = path;
            std::reverse(path.begin(), path.end());
            path_matrix[j][i] = path;
          }
        }
      }

      ROS_INFO("path matrix got, start get cost matrix...");

      std::vector<std::vector<double>> cost_matrix =
          std::vector<std::vector<double>>(
              tour_points_num,
              std::vector<double>(tour_points_num, 10000000.0));
      for (int x = 1; x < tour_points_num; ++x) {
        for (int y = 1; y < tour_points_num; ++y) {
          if (x == y) {
            cost_matrix[x][y] = 0.0;
          } else if (path_matrix[x][y].size() < 2) {
            cost_matrix[x][y] = 10000000;
          } else {
            double path_length = 0.0;
            path_length = path_matrix[x][y].size();
            cost_matrix[x][y] = path_length;
          }
        }
      }
      for (int y = 1; y < tour_points_num; y++) {
        if (path_matrix[0][y].size() < 2) {
          cost_matrix[0][y] = 100000000;
        } else {
          double path_length = 0.0;
          for (int k = 0; k < path_matrix[0][y].size() - 1; k++) {
            path_length = path_length + path_matrix[0][y][k].distance(
                                            path_matrix[0][y][k + 1]);
          }
          cost_matrix[0][y] = path_length;

          if (is_directory_) {
            if (tour_points_term[y].distance(current_position) <
                ufo_map_->max_range_) {
              Point3D diff_vector(
                  tour_points_term[y].x() - current_position.x(),
                  tour_points_term[y].y() - current_position.y(), 0);
              double theta = current_directory.angleTo(diff_vector);
              cost_matrix[0][y] =
                  cost_matrix[0][y] *
                  ((1 - alpha_) * (log(theta / M_PI + 1) / log(2)) + alpha_);
            }
          }
        }
      }
      for (int x = 0; x < tour_points_num; ++x) {
        cost_matrix[x][0] = 0;
      }

      ROS_INFO("cost matrix got, start planning..");

      std::vector<std::vector<int>> nodes_indexs;
      std::vector<int> ids;
      for (int i = 0; i < tour_points_num; ++i) {
        ids.push_back(i);
      }
      ROS_INFO("start 2-opt solver..");
      auto start_time = ros::WallTime::now();

      std::vector<double> gains;
      gains.push_back(0.0);
      for (int i = 1; i < tour_points_num; ++i) {
        gains.push_back(tour_points_gains_[tour_points_term[i]]);
      }

      double lambda = 3.0;
      std::vector<int> init_route = ids;
      Two_Opt two_opt_solve(init_route, gains, cost_matrix, lambda);
      two_opt_solve.solve();
      auto max_unity_way = two_opt_solve.best_route_;

      auto finish_time = ros::WallTime::now();
      double iteration_time = (finish_time - start_time).toSec();
      sum_two_opt_time = sum_two_opt_time + iteration_time;
      std::ofstream fout;
      fout.open(two_opt_time_name_,
                std::ios_base::in | std::ios_base::out | std::ios_base::app);
      fout << solving_num_ << "\t" << iteration_time << "\t"
           << sum_two_opt_time / solving_num_ << "s \t"
           << max_unity_way.size() - 1 << "\t" << std::endl;
      fout.close();

      ROS_INFO("2-opt solver plan is finished, spent %.10f s, finish path node "
               "num is %zu, total searched ways size is %d",
               iteration_time, max_unity_way.size(),
               two_opt_solve.searched_route_num_);
      ROS_INFO("the best route is:");
      for (auto &i : max_unity_way) {
        std::cout << i << "\t";
      }
      std::cout << std::endl;
      ROS_INFO("the best unity is %.10f", two_opt_solve.best_unity_);

      if (max_unity_way.size() > 1) {
        Path tsp_path;
        for (int i = 0; i < max_unity_way.size() - 1; i++) {
          tsp_path.insert(
              tsp_path.end(),
              path_matrix[max_unity_way[i]][max_unity_way[i + 1]].begin(),
              path_matrix[max_unity_way[i]][max_unity_way[i + 1]].end());
          path_segments_.push_back(
              path_matrix[max_unity_way[i]][max_unity_way[i + 1]]);
        }
        tsp_path_ = tsp_path;
        path_to_go_ = path_matrix[0][max_unity_way[1]];
        goal_point_ = tour_points_term.at(max_unity_way[1]);
        ROS_INFO("the goal point is x= %f, y=%f, z=%f", goal_point_.x(),
                 goal_point_.y(), goal_point_.z());
        return true;
      } else {
        return false;
      }
    }
  }

  int HeriGraphCoverPlanner::addCurrentPositionToGraph(
      const Point3D &current_position, graph::PlanGraph &graph) {

    ROS_INFO("add current position to the plan map");
    int current_point_id = graph.addVertex(current_position);

    return current_point_id;
  }

  Path HeriGraphCoverPlanner::getPathInGraph(const int &start_point_id,
                                             const int &end_point_id,
                                             const graph::PlanGraph &graph) {

    Path path;

    return path;
  }

  Path HeriGraphCoverPlanner::getPathInGridMap2D(const Point3D &start_point,
                                               const Point3D &end_point,
                                               const GridMap2D &grid_map_2d) {
    Point2D start_2d(start_point.x(), start_point.y());
    Point2D end_2d(end_point.x(), end_point.y());
    Path path_3d;
    bool is_get_empty = false;
    if (grid_map_2d.isInMapRange2D(start_2d) &&
        grid_map_2d.getStatusInMap2D(start_2d) == Status2D::Free &&
        grid_map_2d.isInMapRange2D(end_2d) &&
        grid_map_2d.getStatusInMap2D(end_2d) == Status2D::Free) {
      ROS_INFO("start search shortest path..");
      std::vector<Point2D> path_2d =
          grid_map_2d.getShortestPath(end_2d, start_2d);
      if (path_2d.size() < 2) {
        is_get_empty = true;
      }
      ROS_INFO("start optimal to straight..");
      auto optimal_path_2d = grid_map_2d.optimalToStraight(path_2d);
      for (const auto &point: optimal_path_2d) {
        path_3d.emplace_back(point.x(), point.y(), global_topo_map_->current_pose_.position.z);
      }
    }
    else {
      ROS_INFO("start point or end point is out of grid map 2d or occupancy");
      path_3d.emplace_back(start_2d.x(), start_2d.y(), global_topo_map_->current_pose_.position.z);
      path_3d.emplace_back(end_2d.x(), end_2d.y(), global_topo_map_->current_pose_.position.z);
    }

    return path_3d;
  }

Path HeriGraphCoverPlanner::MYgetPathInGridMap2D(const Point3D &start_point,
                                             const Point3D &end_point,
                                             const GridMap2D &grid_map_2d,
                                             const GlobalTopoManager::Ptr &topo_manager,
                                             int &approximate_Manhattan_dis) {
  Point2D start_2d(start_point.x(), start_point.y());
  Point2D end_2d(end_point.x(), end_point.y());
  Path path_3d;
  approximate_Manhattan_dis = 0;
  bool is_get_empty = false;
  if (grid_map_2d.isInMapRange2D(start_2d) &&
      grid_map_2d.isInMapRange2D(end_2d)
      ) {
    ROS_INFO("start search shortest path..");
    std::vector<Point2D> path_2d =
        topo_manager->MYgetShortestPath(end_2d, start_2d);
    approximate_Manhattan_dis = path_2d.size();
    if (path_2d.size() < 2) {
      is_get_empty = true;
    }
    ROS_INFO("start optimal to straight..");
    auto optimal_path_2d = topo_manager->MYoptimalToStraight(path_2d);
    for (const auto &point: optimal_path_2d) {
      path_3d.emplace_back(point.x(), point.y(), global_topo_map_->current_pose_.position.z);
    }
  } else {
    ROS_ERROR("start point or end point is out of grid map 2d or occupancy");
    path_3d.clear();
    path_3d.emplace_back(start_2d.x(), start_2d.y(), global_topo_map_->current_pose_.position.z);
    path_3d.emplace_back(end_2d.x(), end_2d.y(), global_topo_map_->current_pose_.position.z);
  }

  return path_3d;
}

double HeriGraphCoverPlanner::getPathLength(const Path &path) {
  double length = 0;
  for (int i = 0; i < path.size() - 1; i++) {
    length += path[i].distanceXY(path[i + 1]);
  }
  return length;
}

}