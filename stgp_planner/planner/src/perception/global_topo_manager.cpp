
#include "perception/global_topo_manager.h"

#include <graph/plan_graph.h>


namespace perception {
GlobalTopoManager::GlobalTopoManager(const ros::NodeHandle &nh,
                                     const ros::NodeHandle &nh_private,
                                     const Ufomap::Ptr &frontier_map)
    : nh_(nh), nh_private_(nh_private), ufomap_(frontier_map) {
  getParamsFromRos();
  init();

  global_topo_grid_pub_ =
    nh_private_.advertise<visualization_msgs::MarkerArray>("global_topo_grid",
                                                             1);
  global_topo_update_timer_ = nh_private_.createTimer(
    ros::Duration(0.1), &GlobalTopoManager::timerCallback, this);

  topo_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("my_topo_markers", 1);
  unknown_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("my_unknown_markers", 1);
  grid_src_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("my_grid_src_markers", 1);
  skeleton_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("my_skeleton_markers", 1);
  full_skeleton_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("my_full_skeleton_markers", 1);

}

void GlobalTopoManager::getParamsFromRos() {
  std::string ns = ros::this_node::getName() + "/GlobalTopoManager";

  grid_size_ = 0.3;
  if (!ros::param::get(ns + "/grid_size", grid_size_)) {
    ROS_WARN("No grid_size specified. Looking for %s. Default is 0.3.",
             (ns + "/grid_size").c_str());
  }

  min_x_ = 0.0;
  if (!ros::param::get(ns + "/min_x", min_x_)) {
    ROS_WARN("No min_x specified. Looking for %s. Default is '0.0'.",
             (ns + "/min_x").c_str());
  }

  min_y_ = 0.0;
  if (!ros::param::get(ns + "/min_y", min_y_)) {
    ROS_WARN("No min_y specified. Looking for %s. Default is '0.0'.",
             (ns + "/min_y").c_str());
  }

  max_x_ = 100.0;
  if (!ros::param::get(ns + "/max_x", max_x_)) {
    ROS_WARN("No max_x specified. Looking for %s. Default is '100.0'.",
             (ns + "/max_x").c_str());
  }

  max_y_ = 100.0;
  if (!ros::param::get(ns + "/max_y", max_y_)) {
    ROS_WARN("No max_y specified. Looking for %s. Default is '100.0'.",
             (ns + "/max_y").c_str());
  }

  wavefront_thresh_ = 4;
  if (!ros::param::get(ns + "/crop_thresh", wavefront_thresh_)) {
    ROS_WARN("No crop_thresh specified. Looking for %s. Default is '6'.",
             (ns + "/crop_thresh").c_str());
  }

  small_branch_len_upper_thresh_ = 6;
  if (!ros::param::get(ns + "/small_branch_len_upper_thresh", small_branch_len_upper_thresh_)) {
    ROS_WARN("No small_branch_len_upper_thresh specified. Looking for %s. Default is '6'.",
             (ns + "/small_branch_len_upper_thresh").c_str());
  }

  small_branch_len_lower_thresh_ = 6;
  if (!ros::param::get(ns + "/small_branch_len_lower_thresh", small_branch_len_lower_thresh_)) {
    ROS_WARN("No small_branch_len_lower_thresh specified. Looking for %s. Default is '6'.",
             (ns + "/small_branch_len_lower_thresh").c_str());
  }

  confluence_step_ = 3;
  if (!ros::param::get(ns + "/confluence_step", confluence_step_)) {
    ROS_WARN("No confluence_step specified. Looking for %s. Default is '3'.",
             (ns + "/confluence_step").c_str());
  }

  std::string pkg_path = ros::package::getPath("planner");
  std::string txt_path = pkg_path + "/../../files/exploration_data/";
  each_grid_update_txt_name_ = txt_path + "each_grid_update_time.txt";
  each_skeleton_txt_name_ = txt_path + "each_skeleton_time.txt";

  std::ofstream fout;

  sum_gridupdate_time_ = 0;
  grid_update_num_ = 0;
  fout.open(each_grid_update_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
  fout << "each grid_update elisped time \n"
       << "start time \t"
       << "end time \t"
       << "elisped time \t"
       << "num \t"
       << "average time \t" << std::endl;
  fout.close();

  sum_skeleton_time_ = 0;
  skeleton_num_ = 0;
  fout.open(each_skeleton_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
  fout << "each skeleton elisped time \n"
       << "start time \t"
       << "end time \t"
       << "elisped time \t"
       << "num \t"
       << "average time \t" << std::endl;
  fout.close();
}

void GlobalTopoManager::init() {
  initGridMap();
  initTopoMap();
}

void GlobalTopoManager::initGridMap() {
  robot_height_ = ufomap_->robot_height_;
  robot_bottom_ = ufomap_->robot_bottom_;
  sensor_height_ = ufomap_->sensor_height_;
  trans_depth_ = ufomap_->frontier_depth_;

  if (grid_size_ != ufomap_->map_.getResolution()) {
    ROS_WARN("grid_size_ is not equal to ufomap resolution");
    grid_size_ = ufomap_->map_.getResolution();
  }
  double x_length = max_x_ - min_x_;
  double y_length = max_y_ - min_y_;
  grid_map_.initialize(grid_size_, x_length, y_length, Empty);
  grid_num_x_ = grid_map_.grid_x_num_;
  grid_num_y_ = grid_map_.grid_y_num_;
  Point2D center = Point2D(round((max_x_ + min_x_)*0.5), round((max_y_+min_y_)*0.5));
  grid_map_.setMapCenterAndBoundary(center);

  resetChangedboundary();
}

void GlobalTopoManager::initTopoMap() {
  grid_map_image_ = cv::Mat::ones(grid_num_y_, grid_num_x_, CV_8UC1) * 128;
  wavefront_ = cv::Mat::ones(grid_num_y_, grid_num_x_, CV_32SC1) * -1;
  removed_free_mask_ = cv::Mat::zeros(grid_num_y_, grid_num_x_, CV_8UC1);
  skeleton_mask_ = cv::Mat::zeros(grid_num_y_, grid_num_x_, CV_8UC1);
  unconfirmed_skeleton_mask_ = cv::Mat::zeros(grid_num_y_, grid_num_x_, CV_8UC1);
  unknown_mask_ = cv::Mat::ones(grid_num_y_, grid_num_x_, CV_8UC1) * 255;
}

void GlobalTopoManager::timerCallback(const ros::TimerEvent &event) {
  ROS_DEBUG("timer test");
  ufomap_->frontierUpdate();
  updateGridMap();
}

void GlobalTopoManager::updateGridMap() {
  global_topo_update_mutex_.lock();

  ros::WallTime gridupate_start_time = ros::WallTime::now();

  ufomap_->odom_mutex_.lock();
  ufo::math::Vector3 current = ufomap_->current_robot_pose_.translation();
  updateRobotPose(current.x(), current.y(), current.z());
  ufomap_->odom_mutex_.unlock();

  for (const auto &changed_cell_code : ufomap_->all_changed_cell_codes_) {
    if (changed_cell_code.getDepth() <= 3) {
      ufo::map::Point3 point = ufomap_->map_.toCoord(changed_cell_code.toKey());
      if (isInGridArea(point.x(), point.y()) &&
          point.z() > ufomap_->current_robot_pose_.z() + robot_bottom_ &&
          point.z() < ufomap_->current_robot_pose_.z() + robot_height_) {
        const UfoState ufomap_state = ufomap_->map_.getState(changed_cell_code);
        for (auto &grid_id: cell2Index(Point2D(point.x(), point.y()), changed_cell_code.getDepth())) {
          const UfoState ufomap_state = ufomap_->map_.getState(changed_cell_code);
          const GridState grid_state = grid_map_.getStatusInMap2D(grid_id);
          if (ufomap_state == UfoState::occupied && grid_state != Occupied) {
            grid_map_.setOccupied(grid_id);
            setMatState(grid_id, Occupied);
            // updateChangedGrid(grid_id);
          } else if (ufomap_state == UfoState::free && grid_state != Occupied  && grid_state != Free) {
            grid_map_.setFree(grid_id);
            setMatState(grid_id, Free);
            // updateChangedGrid(grid_id);
          } else if (ufomap_state == UfoState::unknown) {
          }
        }
      }
    }
  }

  const cv::Point min_point = index2CvPoint(changed_grid_min_);
  const cv::Point max_point = index2CvPoint(changed_grid_max_);
  const cv::Rect roi(0, 0, grid_map_image_.cols, grid_map_image_.rows);
  if (roi.height > 0 && roi.width > 0) {
    cv::Mat grid_map_image_roi = grid_map_image_(roi);
    cv::Mat inflate_grid_map = grid_map_image_roi.clone();
    dilateSpecificValue(inflate_grid_map, 0);
    dilateSpecificValue(inflate_grid_map, 255);
    dilateSpecificValue(inflate_grid_map, 128);
    cv::Mat mask_unknown = (grid_map_image_roi == 128);
    inflate_grid_map.copyTo(grid_map_image_roi, mask_unknown);
  }

  ros::WallTime skeleton_start_time = ros::WallTime::now();
  if (is_global_topo_grid_updated_) {
    updateSkeleton();
    is_global_topo_grid_updated_ = false;
  }
  ros::WallTime end_time = ros::WallTime::now();

  grid_update_num_++;
  sum_gridupdate_time_ = sum_gridupdate_time_ + (skeleton_start_time - gridupate_start_time).toSec();
  std::ofstream fout;
  fout.open(each_grid_update_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::app);
  fout << gridupate_start_time << "\t" << skeleton_start_time << "\t"
       << (skeleton_start_time - gridupate_start_time).toSec() << "\t" << grid_update_num_ << "\t"
       << sum_gridupdate_time_ / grid_update_num_ << "\t" << std::endl;
  fout.close();

  skeleton_num_++;
  sum_skeleton_time_ = sum_skeleton_time_ + (end_time - skeleton_start_time).toSec();
  fout.open(each_skeleton_txt_name_,
            std::ios_base::in | std::ios_base::out | std::ios_base::app);
  fout << skeleton_start_time << "\t" << end_time << "\t"
       << (end_time - skeleton_start_time).toSec() << "\t" << skeleton_num_ << "\t"
       << sum_skeleton_time_ / skeleton_num_ << "\t" << std::endl;
  fout.close();


  global_topo_update_mutex_.unlock();

}

void GlobalTopoManager::getRoiForThin() {
  roi_ = cv::Rect(0, 0, grid_num_x_, grid_num_y_);
  roi_pt_ = cv::Point(roi_.x, roi_.y);
}

void GlobalTopoManager::updateSkeleton() {
  getRoiForThin();
  const cv::Mat src_roi = grid_map_image_(roi_);
  cv::Mat wavefront_roi = wavefront_(roi_);
  cv::Mat removed_roi = removed_free_mask_(roi_);
  skeleton_mask_roi_ = skeleton_mask_(roi_);
  cv::Mat unconfirmed_skeleton_roi = unconfirmed_skeleton_mask_(roi_);
  cv::Mat unknwon_mask_roi = unknown_mask_(roi_);
  cv::Mat binary_roi;
  cv::threshold(src_roi, binary_roi, 129, 255, cv::THRESH_BINARY);
  binary_roi = binary_roi - removed_roi;
  incremental_thin_flag_ = incrementalThinning(binary_roi, src_roi, skeleton_mask_roi_, unknwon_mask_roi, removed_roi,
                                           wavefront_roi, unconfirmed_skeleton_roi, true);
  time_cout_ += 1;


  // unknown_pub_.publish(generateOpencvCvMarkers(unknown_mask_, true, -1));
  // grid_src_pub_.publish(generateOpencvCvMarkers(grid_map_image_, false, -1.5));
  // skeleton_pub_.publish(generateOpencvCvMarkers(skeleton_mask_, true, -2));
  // full_skeleton_pub_.publish(generateOpencvCvMarkers(unconfirmed_skeleton_mask_, true, -2.5));
}

void GlobalTopoManager::updateTopo() {
  cv::filter2D(skeleton_mask_roi_, connection_num_roi_, CV_8U, conn_kernal_);
  cv::waitKey(1);
  connection_num_roi_.convertTo(connection_num_roi_, CV_8S);
  connection_num_roi_ -= 8;

  if (!initHome_) setHome(robot_current_id_);
  if (!initTopo_ && incremental_thin_flag_) {
    VertexDescriptor home_node = boost::add_vertex(home_node_label_, heriTopo_);
    heriTopo_[home_node_label_].generate_time = time_cout_;
    heriTopo_[home_node_label_].cvPt = index2CvPoint(getHomeIndex());
    heriTopo_[home_node_label_].type = Termination;
    heriTopo_[home_node_label_].parent_branch_label = home_node_label_;
    heriTopo_[home_node_label_].parent_label = home_node_label_;
    heriTopo_[home_node_label_].parent_junction_label = home_node_label_;
    heriTopo_[home_node_label_].dis_to_parent = 0;
    int min_distance = 10000;
    int nearest_label;
    Index2D nearest_index;
    if (skeleton_mask_.at<uint8_t>(index2CvPoint(home_index2D_))) {
      nearest_index = home_index2D_;
      min_distance = 0;
      nearest_label = home_node_label_;
    }
    else {
      for (int row = 0; row < skeleton_mask_roi_.rows; ++row) {
        for (int col = 0; col < skeleton_mask_roi_.cols; ++col) {
          if (skeleton_mask_roi_.at<uint8_t>(row, col) != 0 &&
            connection_num_roi_.at<int8_t>(row, col) > 0) {
            auto index = Index2D(col+roi_.x, row+roi_.y);
            int distance = squaredIdDistance(index, home_index2D_);
            if (min_distance > distance) {
              min_distance = distance;
              nearest_index = index;
            }
            }
        }
      }
      nearest_label = index2NodeLabel(nearest_index);
      double dis_to_parent = sqrt(min_distance);
      if (unknown_mask_.at<uchar>(index2CvPoint(nearest_index)) > 0)
        return;
      VertexDescriptor nearest_node = boost::add_vertex(nearest_label, heriTopo_);
      heriTopo_[nearest_label].generate_time = time_cout_;
      heriTopo_[nearest_label].type = Branch_junction;
      heriTopo_[nearest_label].dis_to_parent =
        static_cast<int>(std::floor(dis_to_parent + 0.5));
      heriTopo_[home_node_label_].children_labels.push_back(nearest_label);
    }
    heriTopo_[nearest_label].cvPt = index2CvPoint(nearest_index);
    heriTopo_[nearest_label].parent_label = home_node_label_;
    heriTopo_[nearest_label].parent_branch_label = home_node_label_;
    heriTopo_[nearest_label].parent_junction_label = home_node_label_;

    initTopo_ = true;

    const Index2D c_id = nodeLabel2Index(nearest_label);
    const cv::Point c_pt = index2CvPoint(c_id);
    for (int row = -1; row <= 1; ++row) {
      for (int col = -1; col <= 1; ++col) {
        if (row == 0 && col == 0) continue;
        const cv::Point n_pt = c_pt + cv::Point(col, row);
        if (!isInRoi(n_pt)) continue;
        if (skeleton_mask_.at<uchar>(n_pt)) {
          const int n_label = cvPoint2NodeLabel(n_pt);
          VertexDescriptor n_node = boost::add_vertex(n_label, heriTopo_);
          heriTopo_[n_label].generate_time = time_cout_;
          heriTopo_[n_label].type = Termination;
          heriTopo_[n_label].cvPt = n_pt;
          heriTopo_[n_label].parent_label = nearest_label;
          heriTopo_[n_label].parent_branch_label = n_label;
          heriTopo_[n_label].parent_junction_label = nearest_label;
          heriTopo_[n_label].dis_to_parent = 1;

          heriTopo_[nearest_label].children_labels.push_back(n_label);
          local_frontier_labels_.push_back(n_label);
        }
      }
    }
  }

  auto local_frontier_labels_copy = local_frontier_labels_;
  local_frontier_labels_.clear();
  for (auto &c_labbel: local_frontier_labels_copy) {
    const Index2D c_id = nodeLabel2Index(c_labbel);
    const cv::Point c_pt = index2CvPoint(c_id);
    if (!isInRoi(c_pt)) {
      local_frontier_labels_.push_back(c_labbel);
      continue;
    }
    int distance = heriTopo_[c_labbel].dis_to_parent;
    ROS_INFO("Update children nodes-----------------");
    updateNodesRecursion(c_labbel, c_pt, distance);
  }



  pubTopoMarkers();
  global_topo_grid_pub_.publish(
      grid_map_.generateMapMarkers(grid_map_.grids_, current_pose_));
}

void GlobalTopoManager::updateNodesRecursion(const int &center_label,
                                            const cv::Point& center_cvPoint,
                                            int distance) {
  const bool adj_to_unknown = unknown_mask_.at<uchar>(center_cvPoint) > 0;
  const int c_conn_num = connection_num_roi_.at<int8_t>(center_cvPoint - roi_pt_);
  if (c_conn_num <= 0) {
    ROS_ERROR("NOT SKELETON POINT! %d -- %d -- %d", heriTopo_[center_label].type, c_conn_num, center_label);
  }
  assert(!heriTopo_[center_label].removed);
  const int parent_label = heriTopo_[center_label].parent_label;
  int parent_branch_label = heriTopo_[center_label].parent_branch_label;
  bool parent_branch_label_as_self = false;
  int parent_junction_label = heriTopo_[center_label].parent_junction_label;
  std::vector<int> new_children_labels;
  int c_generate_time = heriTopo_[center_label].generate_time;
  bool is_history_node = c_generate_time < time_cout_;
  bool check_neighbor = false;

  assert(heriTopo_[center_label].generate_time <= time_cout_);
  if (is_history_node) {
    assert(!heriTopo_[center_label].confirmed);
    if (heriTopo_[center_label].conn_num != c_conn_num) {
      check_neighbor = true;
    }
  }
  else check_neighbor = true;

  if (check_neighbor) {
    for (const auto& pt: neighbor8_cvPt) {
      cv::Point n_pt = center_cvPoint + pt;
      if (!isInRoi(n_pt)) continue;
      int n_label = cvPoint2NodeLabel(n_pt);
      if (n_label == parent_label) continue;
      int n_conn_num = connection_num_roi_.at<int8_t>(n_pt - roi_pt_);
      if (n_conn_num > 0) {
        if (std::find(heriTopo_[center_label].children_labels.begin(),
                      heriTopo_[center_label].children_labels.end(),
                      n_label) != heriTopo_[center_label].children_labels.end()) {
          continue;
                      }
        VertexDescriptor n_des = boost::add_vertex(n_label, heriTopo_);
        int n_gen_time = heriTopo_[n_label].generate_time;
        if ( n_gen_time != -1) {
          if (heriTopo_[n_label].removed)
            continue;
          if (n_gen_time != c_generate_time) {
            proc_confluence_labels_.push_back(n_label);
            heriTopo_[n_label].unconfirmed_father_labels.insert(center_label);
            continue;
          } else continue;
        }
        heriTopo_[n_label].generate_time = time_cout_;
        heriTopo_[n_label].cvPt = n_pt;
        new_children_labels.push_back(n_label);
        if (n_conn_num == 1) {
          heriTopo_[n_label].type = Termination;
          heriTopo_[n_label].conn_num = 1;
          if (unknown_mask_.at<uchar>(n_pt) > 0) {
            heriTopo_[n_label].confirmed = false;
            local_frontier_labels_.push_back(n_label);
          }
          else crop_termination_labels_.push_back(n_label);
        }
      }
    }
  }

  int c_children_num = new_children_labels.size();
  if (is_history_node) {
    NodeType ori_type = heriTopo_[center_label].type;
    if (c_children_num == 0 && check_neighbor && ori_type == Termination) {
      heriTopo_[center_label].type = FakeTermination;
    }
    else if (c_children_num == 1) {
      if (ori_type == Termination || ori_type == FakeTermination) {
        heriTopo_[center_label].type = Connection;
      }
      else if (ori_type == Connection) {
        heriTopo_[center_label].type = Branch_junction;
        parent_branch_label_as_self = true;
        parent_junction_label = center_label;
        distance = 0;
      }
    }
    else if (c_children_num > 1) {
      heriTopo_[center_label].type = Branch_junction;
      parent_branch_label_as_self = true;
      parent_junction_label = center_label;
      distance = 0;
    }
  }
  else {
    if (c_children_num == 0) {
      if (c_conn_num == 1) {
        heriTopo_[center_label].type = Termination;
      } else {
        heriTopo_[center_label].type = FakeTermination;
      }
    } else if (c_children_num == 1) {
      heriTopo_[center_label].type = Connection;
    } else if (c_children_num > 1) {
      heriTopo_[center_label].type = Branch_junction;
      distance = 0;
      parent_branch_label_as_self = true;
      parent_junction_label = center_label;
    }
  }

  for (auto n_label: new_children_labels) {
    heriTopo_[n_label].parent_label = center_label;
    if (parent_branch_label_as_self) {
      heriTopo_[n_label].parent_branch_label = n_label;
    } else {
      heriTopo_[n_label].parent_branch_label = parent_branch_label;
    }
    heriTopo_[n_label].parent_junction_label = parent_junction_label;
    heriTopo_[n_label].dis_to_parent = distance + 1;
    if (heriTopo_[n_label].type == Termination) continue;
    updateNodesRecursion(n_label, heriTopo_[n_label].cvPt, distance + 1);
  }

  if (adj_to_unknown) {
    heriTopo_[center_label].confirmed = false;
    local_frontier_labels_.push_back(center_label);
    if (is_history_node) updateHistoryChildren(center_label);
  }
  else {
    heriTopo_[center_label].confirmed = true;
    if (is_history_node) updateHistoryChildren(center_label);
    if (heriTopo_[center_label].type == Termination || heriTopo_[center_label].type == FakeTermination) {
      crop_termination_labels_.push_back(center_label);
    }
  }
  heriTopo_[center_label].conn_num = c_conn_num;
  heriTopo_[center_label].children_labels.insert(
    heriTopo_[center_label].children_labels.end(),
    new_children_labels.begin(), new_children_labels.end());

  if (c_conn_num == 1) {
    if (heriTopo_[center_label].type != Termination) {
      ROS_ERROR("All the nodes in frontier should be T or FT, something wrong! - %d",heriTopo_[center_label].type);
      for (auto n_label: heriTopo_[center_label].children_labels) {
        ROS_ERROR("child %d", n_label);
      }
    }
  }
}

void GlobalTopoManager::updateHistoryChildren(const int &center_label) {
  if (heriTopo_[center_label].type == Branch_junction) {
    if (heriTopo_[center_label].children_labels.size() == 1) {
      int parent_junction_label = center_label;
      int distance = 0;
      for (auto child_label: heriTopo_[center_label].children_labels) {
        Node parent_branch_label = child_label;
        updateHistoryChildrenRec(child_label, center_label, parent_branch_label, parent_junction_label, distance+1);
      }
    }
  }
  else if (heriTopo_[center_label].type == Confluence_junction) {
  }
}

void GlobalTopoManager::updateHistoryChildrenRec(int center_label, int parent_label, int parent_branch_label, int parent_junction_label, int distance) {
  heriTopo_[center_label].parent_label = parent_label;
  heriTopo_[center_label].parent_branch_label = parent_branch_label;
  heriTopo_[center_label].parent_junction_label = parent_junction_label;
  heriTopo_[center_label].dis_to_parent = distance;
  if (heriTopo_[center_label].type == Connection) {
    assert(heriTopo_[center_label].children_labels.size() == 1);
    updateHistoryChildrenRec(heriTopo_[center_label].children_labels[0], center_label, parent_branch_label, parent_junction_label, distance+1);
  }
}

bool GlobalTopoManager::haveOverlap(const std::unordered_set<int>& set1, const std::unordered_set<int>& set2) {
  for (int elem : set1) {
    if (set2.find(elem) != set2.end()) {
      return true;
    }
  }
  return false;
}

void GlobalTopoManager::updateChangedGrid(const Index2D &grid_id) {
  if (grid_id.x() < changed_grid_min_.x()) changed_grid_min_.x() = grid_id.x();
  else if (grid_id.x() > changed_grid_max_.x()) changed_grid_max_.x() = grid_id.x();

  if (grid_id.y() < changed_grid_min_.y()) changed_grid_min_.y() = grid_id.y();
  else if (grid_id.y() > changed_grid_max_.y()) changed_grid_max_.y() = grid_id.y();
}

void GlobalTopoManager::getChangedboundary() {
}

bool GlobalTopoManager::isInGridArea(const double &point_x,
                                     const double &point_y) const {
  if (point_x < min_x_ + 1e-4 || point_x > max_x_ - 1e-4 ||
      point_y < min_y_ + 1e-4 || point_y > max_y_ - 1e-4) {
    return false;
      }
  return true;
}

bool GlobalTopoManager::isInGridArea(const Index2D &index) const {
  Point2D point_xy = index2Point(index);
  return isInGridArea(point_xy(0), point_xy(1));
}

void GlobalTopoManager::dilateSpecificValue(cv::Mat &image, int value) const {
  cv::Mat mask = (image == value);
  cv::Mat dilatedMask;

  dilate(mask, dilatedMask, kernal_);

  image.setTo(value, dilatedMask);
}


void GlobalTopoManager::generateTopo() {
}

bool GlobalTopoManager::incrementalThinning(const cv::Mat &binary, const cv::Mat &src, cv::Mat &confirmed_skeleton,cv::Mat &unknown,cv::Mat &remove, cv::Mat &wavefront_count, cv::Mat &unconfirmed_skeleton, const bool check_unknown)const {
  cv::Mat for_vis = src.clone();
  cv::Mat remove_copy = remove.clone();

  int nrows = binary.rows + 2;
  int ncols = binary.cols + 2;
  cv::Mat skeleton = cv::Mat::zeros(nrows, ncols, CV_8UC1);
  binary.copyTo(skeleton(cv::Rect(1, 1, binary.cols, binary.rows)));
  skeleton = skeleton/255;
  cv::Mat cleaned_skeleton = skeleton.clone();

  cv::Mat wavefront_count_copy = cv::Mat::zeros(nrows, ncols, CV_32SC1);
  wavefront_count.copyTo(wavefront_count_copy(cv::Rect(1, 1, binary.cols, binary.rows)));

  cv::Mat neibor = skeleton.clone();

  cv::Mat colormap_src;
  cv::Mat occupied_mask, occupied_mask_pad;
  occupied_mask = (for_vis == 0);
  occupied_mask_pad = cv::Mat::zeros(nrows, ncols, CV_8UC1);
  occupied_mask.copyTo(occupied_mask_pad(cv::Rect(1, 1, binary.cols, binary.rows)));

  cv::Mat unknown_mask, unknown_mask_pad;
  unknown_mask = (for_vis == 128);
  unknown_mask_pad = cv::Mat::zeros(nrows, ncols, CV_8UC1);
  unknown_mask.copyTo(unknown_mask_pad(cv::Rect(1, 1, binary.cols, binary.rows)));
  cv::Mat unknown_mask_pad_dilate = unknown_mask_pad;



  bool pixel_removed = true;
  int count = 0;
  int remove_count = 0;
  cv::Mat unknown_match;
  while (pixel_removed) {
    pixel_removed = false;
    for (int pass_num = 0; pass_num < 2; ++pass_num) {
      cv::Mat cond1 = (skeleton == 1);
      cv::Mat cond2 = (skeleton == 0);
      cv::Mat cond3;
      cv::compare(wavefront_count_copy, count, cond3, cv::CmpTypes::CMP_GE);
      cv::Mat cond23 = cond2 & cond3;
      neibor = cond1 | cond23;
      neibor /= 255;

      if (check_unknown) {
        cv::dilate(unknown_mask_pad_dilate, unknown_match, kernal_);
      };
      bool unknown_neighbor = false;
      bool first_pass = (pass_num == 0);
      for (int row = 1; row < nrows - 1; ++row) {
        for (int col = 1; col < ncols - 1; ++col) {
          if (skeleton.at<uint8_t>(row, col) != 0) {
            int neighbors = neibor.at<uint8_t>(row - 1, col - 1)
                            + 2 * neibor.at<uint8_t>(row - 1, col)
                            + 4 * neibor.at<uint8_t>(row - 1, col + 1)
                            + 8 * neibor.at<uint8_t>(row, col + 1)
                            + 16 * neibor.at<uint8_t>(row + 1, col + 1)
                            + 32 * neibor.at<uint8_t>(row + 1, col)
                            + 64 * neibor.at<uint8_t>(row + 1, col - 1)
                            + 128 * neibor.at<uint8_t>(row, col - 1);
            uint8_t lut_value = lut[neighbors];

            if (lut_value == 0) continue;
            if (lut_value == 3 || (lut_value == 1 && first_pass) || (lut_value == 2 && !first_pass)) {
              cleaned_skeleton.at<uint8_t>(row, col) = 0;
              pixel_removed = true;
              if (check_unknown) {
                if (unknown_match.at<uint8_t>(row, col) == 0) {
                  wavefront_count_copy.at<int>(row, col) = count;
                  for_vis.at<uint8_t>(row-1, col-1) = 255;
                  remove_copy.at<uchar>(row-1, col-1) = 255;
                  remove_count += 1;
                }
                else {
                  unknown_mask_pad_dilate.at<uint8_t>(row, col) = 255;
                  for_vis.at<uint8_t>(row-1, col-1) = 128;
                }
              }
              else for_vis.at<uint8_t>(row-1, col-1) = 255;
            }
          }
        }
      }
      skeleton = cleaned_skeleton.clone();
      count += 1;
    }
    std::cout<<count<<std::endl;


  }
  if (true) {

    unknown_mask_pad_dilate = unknown_mask_pad_dilate(cv::Rect(1, 1, binary.cols, binary.rows));
    cv::dilate(unknown_mask_pad_dilate, unknown_match, kernal_);
    unknown_match.setTo(0, occupied_mask==255);
    skeleton = skeleton(cv::Rect(1, 1, binary.cols, binary.rows));
    cv::Mat new_skeleton = skeleton & unknown_match == 0;
    new_skeleton = new_skeleton | confirmed_skeleton;
    new_skeleton.copyTo(confirmed_skeleton);
    skeleton = skeleton | confirmed_skeleton;
    skeleton.copyTo(unconfirmed_skeleton);

    cv::dilate(unknown_match, unknown_match, kernal_);


    unknown_match.copyTo(unknown);

    remove_copy = (remove_copy & (unknown_match == 0));
    remove_copy.copyTo(remove);

    wavefront_count_copy = wavefront_count_copy(cv::Rect(1, 1, binary.cols, binary.rows));
    wavefront_count_copy.copyTo(wavefront_count);
    return true;
  }
  return false;

}

Node GlobalTopoManager::findNearestNode(double x, double y) {
  int graph_size = boost::num_vertices(heriTopo_);
  Node center_node = index2NodeLabel(point2Index(Point2D{x,y}));
  if (boost::vertex_by_label(center_node, heriTopo_) < graph_size) {
    assert(heriTopo_[center_node].parent_label != -1);
    return center_node;
  }

  NodeSet visited_node;
  std::vector visit_order = {500,1,-500,-1};
  Node nearest_node = -1;
  NodeQueue need_to_check;
  visited_node.insert(center_node);
  need_to_check.push_back(center_node);
  while(!need_to_check.empty()) {
    for (auto &node: need_to_check) {
      for (auto &order: visit_order) {
        Node next_node = node + order;
        if (visited_node.find(next_node) != visited_node.end()) continue;
        if (boost::vertex_by_label(next_node, heriTopo_) < graph_size) {
          assert(heriTopo_[next_node].parent_label != -1);
          nearest_node = next_node;
          break;
        }
        visited_node.insert(next_node);
        need_to_check.push_back(next_node);
      }
    }
  }
  return nearest_node;
}

void GlobalTopoManager::updateLocalFrontier(int current_explore_branch) {
  assert(local_frontier_labels_.empty());
  Node parent_junction = heriTopo_[current_explore_branch].parent_junction_label;
  auto &frontiers =
      heriTopo_[parent_junction].children_with_unprocessed_frontier[current_explore_branch];
  for (auto frontier_it = frontiers.begin(); frontier_it != frontiers.end();) {
    const Node frontier = *frontier_it;
    if (heriTopo_[frontier].removed || heriTopo_[frontier].confirmed) {
      ROS_WARN("drop stale local frontier label %d during frontier update", frontier);
      frontier_it = frontiers.erase(frontier_it);
      continue;
    }
    local_frontier_labels_.push_back(frontier);
    ++frontier_it;
  }
}

bool GlobalTopoManager::frontierClassify(int current_explore_branch, std::vector<MissedBranchMap> &history_missed_branches, bool &exist_missed_branches) {
  if (local_frontier_labels_.empty()) {
    exist_missed_branches = false;
    return false;
  }

  bool temp_test = true;
  MissedBranchMap temp_missed_branches;

  bool exist_frontier_in_current_explore_branch = false;
  NodeQueue global_frontier_labels;
  int current_explore_parent_junction_label = heriTopo_[current_explore_branch].parent_junction_label;
  NodeQueue local_frontier_labels_copy = local_frontier_labels_;
  local_frontier_labels_.clear();
  NodeSet branches_need_to_backtracking;
  for (auto frontier_label: local_frontier_labels_copy) {
    NodeData local_frontier = heriTopo_[frontier_label];
    if (local_frontier.removed || local_frontier.confirmed) {
      ROS_WARN("drop stale frontier label %d during frontier classify", frontier_label);
      continue;
    }
    int parent_junction_label = local_frontier.parent_junction_label;
    int parent_branch_label = local_frontier.parent_branch_label;
    if (parent_branch_label == current_explore_branch) {
      local_frontier_labels_.push_back(frontier_label);
      exist_frontier_in_current_explore_branch = true;
    }
    else if (exist_frontier_in_current_explore_branch) {
      global_frontier_labels.push_back(frontier_label);
      continue;
    }
    else {
      temp_missed_branches[parent_branch_label].frontiers.insert(frontier_label);
      global_frontier_labels.push_back(frontier_label);
      heriTopo_[parent_junction_label].children_with_unprocessed_frontier[parent_branch_label].insert(frontier_label);
      branches_need_to_backtracking.insert(parent_branch_label);
    }
  }

  if (global_frontier_labels.empty()) exist_missed_branches = false;
  else exist_missed_branches = true;

  if (exist_frontier_in_current_explore_branch) {
    if (exist_missed_branches)
      local_frontier_labels_.insert(local_frontier_labels_.end(), global_frontier_labels.begin(), global_frontier_labels.end());
    return true;
  }

  if (temp_test) {
      if (exist_missed_branches) {
        history_missed_branches.push_back(temp_missed_branches);
      }
      return false;
  }
  else {

  if (exist_missed_branches) {
    MissedBranchMap missed_branches;
    for (auto &branch: branches_need_to_backtracking) {
      Node current_junction = heriTopo_[branch].parent_junction_label;
      while (true) {
        NodeData current_junction_data = heriTopo_[current_junction];
        if (current_junction_data.type == Confluence_junction) {
        }
        else {
          Node current_junction_parent_junction = current_junction_data.parent_junction_label;
          missed_branches[branch].path.push_back(current_junction);
          if (current_junction_parent_junction == current_explore_parent_junction_label) {
            break;
          }
          missed_branches[branch].distance += current_junction_data.dis_to_parent;
          current_junction = current_junction_data.parent_junction_label;
        }
      }
    }


    Node parent_junction = -1;
    for (auto &item: missed_branches) {
      if (parent_junction == -1) {
        parent_junction = item.second.path.back();
      }
      else {
        if (parent_junction != item.second.path.back()) {
          ROS_ERROR("missed_branches not from the same junction~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        }
      }
    }
    history_missed_branches.push_back(missed_branches);
  }
  return false;
  }
}

void GlobalTopoManager::backtrackingForConfluenceJunction() {
}


void GlobalTopoManager::updateCurrentExploreBranch(int &current_explore_branch, std::vector<MissedBranchMap> &history_missed_branches, NodeQueue &passed_junctions) {
  assert(local_frontier_labels_.empty());
  MissedBranchMap missed_branches = history_missed_branches.back();
  MissedBranchMap missed_branches_copy;
  Node nearest_branch = -1;
  Node nearest_frontier = -1;
  int min_distance_frontier = 10000;
  for (auto &item: missed_branches) {
    for (auto &frontier: item.second.frontiers) {
      if (heriTopo_[frontier].removed || heriTopo_[frontier].confirmed) {
        ROS_WARN("drop stale missed frontier label %d while changing branch", frontier);
        continue;
      }
      Node parent_branch_node = heriTopo_[frontier].parent_branch_label;
      missed_branches_copy[parent_branch_node].frontiers.insert(frontier);
      int dis_from_current_2_frontier = squaredIdDistance(robot_current_id_, nodeLabel2Index(frontier));
      if (dis_from_current_2_frontier < min_distance_frontier) {
        min_distance_frontier = dis_from_current_2_frontier;
        nearest_frontier = frontier;
        nearest_branch = parent_branch_node;
      }
    }
    
  }
  if (nearest_branch == -1) {
    ROS_WARN("all missed frontiers are stale while changing branch");
    history_missed_branches.pop_back();
    local_frontier_labels_.clear();
    return;
  }


  assert(local_frontier_labels_.empty());
  
  local_frontier_labels_.insert(local_frontier_labels_.end(),
    missed_branches_copy[nearest_branch].frontiers.begin(),
    missed_branches_copy[nearest_branch].frontiers.end());
  current_explore_branch = nearest_branch;

  history_missed_branches.pop_back();
  missed_branches_copy.erase(nearest_branch);
  if (!missed_branches_copy.empty()){
    history_missed_branches.push_back(missed_branches_copy);
  }
}





bool GlobalTopoManager::localFrontierRelocate(int &current_explore_branch, std::vector<MissedBranchMap> &history_missed_branches, NodeQueue &path) {
  if (history_missed_branches.empty()) return false;
  Node nearest_branch;
  Node nearest_frontier;
  int min_distance_frontier;
  bool is_relocated = false;
  MissedBranchMap missed_branches = history_missed_branches.back();
  while (!is_relocated) {
    MissedBranchMap missed_branches_copy;
    nearest_branch = -1;
    nearest_frontier = -1;
    min_distance_frontier = 100000;
    for (auto &item: missed_branches) {
      for (auto &frontier: item.second.frontiers) {
        if (heriTopo_[frontier].removed || heriTopo_[frontier].confirmed) {
          ROS_WARN("drop stale missed frontier label %d during relocation", frontier);
          continue;
        }
        Node parent_branch_node = heriTopo_[frontier].parent_branch_label;
        missed_branches_copy[parent_branch_node].frontiers.insert(frontier);
        int dis_from_current_2_frontier = squaredIdDistance(robot_current_id_, nodeLabel2Index(frontier));
        if (dis_from_current_2_frontier < min_distance_frontier) {
          min_distance_frontier = dis_from_current_2_frontier;
          nearest_frontier = frontier;
          nearest_branch = parent_branch_node;
        }
      }
    }

    if (nearest_branch == -1) {
      ROS_WARN("all missed frontiers are stale during relocation");
      history_missed_branches.pop_back();
      local_frontier_labels_.clear();
      return false;
    }
    history_missed_branches.pop_back();


    assert(local_frontier_labels_.empty());
    local_frontier_labels_.insert(local_frontier_labels_.end(),
      missed_branches_copy[nearest_branch].frontiers.begin(),
      missed_branches_copy[nearest_branch].frontiers.end());

    missed_branches_copy.erase(nearest_branch);

    updateTopo();

    if (local_frontier_labels_.empty()) {
      MissedBranchMap missed_branches_copy_copy = missed_branches_copy;
      for (auto &item: missed_branches_copy) {
        local_frontier_labels_.insert(local_frontier_labels_.end(),
          item.second.frontiers.begin(),
          item.second.frontiers.end());
        updateTopo();
        if (!local_frontier_labels_.empty()) {
          current_explore_branch = item.first;
          path.clear();
          path.push_back(item.first);
          missed_branches_copy_copy.erase(item.first);
          if (!missed_branches_copy_copy.empty()) {
            history_missed_branches.push_back(missed_branches_copy_copy);
          }
          return true;
        }
        else {
          missed_branches_copy_copy.erase(item.first);
        }
      }
      if (!missed_branches_copy_copy.empty()) {
        history_missed_branches.push_back(missed_branches_copy_copy);
      }
      else return false;
    }
    else {
      current_explore_branch = nearest_branch;
      path.clear();
      path.push_back(nearest_branch);
      if (!missed_branches_copy.empty()) {
        history_missed_branches.push_back(missed_branches_copy);
      }
      return true;
    }
  }
}




Node GlobalTopoManager::extendUnconfirmedFrontier(Node current_explore_branch, Node current_frontier, int lookadhead_thre)
{
  assert (current_explore_branch == heriTopo_[current_frontier].parent_branch_label);
  Node parent = heriTopo_[current_frontier].parent_label;
  cv::Point parent_cvpt = nodeLabel2CvPoint(current_frontier);
  cv::Point last_parent_cvpt = nodeLabel2CvPoint(parent);

  int lookahead_dis = 0;
  std::vector<cv::Point> children;
  while (true)
  {
    if (lookahead_dis > lookadhead_thre) break;

    children.clear();
    for (const auto& shift: neighbor8_cvPt)
    {
      cv::Point child_cvpt = parent_cvpt + shift;
      if (child_cvpt == last_parent_cvpt) continue;
      if (unconfirmed_skeleton_mask_.at<uint8_t>(child_cvpt) == 1)
      {
        children.push_back(child_cvpt);
      }
    }
    if (children.size() == 0) break;
    if (children.size() == 1)
    {
      lookahead_dis += 1;
      last_parent_cvpt = parent_cvpt;
      parent_cvpt = children.back();
    }
    else
    {
      break;
    }
  }

  Node target = cvPoint2NodeLabel(parent_cvpt);

  return target;
}

visualization_msgs::MarkerArray GlobalTopoManager::generateTopoMarkers() const {
  visualization_msgs::Marker termination;
  visualization_msgs::Marker faketermination;
  visualization_msgs::Marker connection;
  visualization_msgs::Marker junctions;
  visualization_msgs::Marker confluence;

  visualization_msgs::Marker edges;
  visualization_msgs::Marker edges2;

  visualization_msgs::Marker skeleton;

  termination.header.frame_id = faketermination.header.frame_id =
    connection.header.frame_id = junctions.header.frame_id = confluence.header.frame_id =
      edges.header.frame_id = edges2.header.frame_id =
        skeleton.header.frame_id = "map";
  termination.header.stamp = faketermination.header.stamp =
    connection.header.stamp = junctions.header.stamp = confluence.header.stamp =
      edges.header.stamp = edges2.header.stamp =
        skeleton.header.stamp = ros::Time::now();
  termination.ns = faketermination.ns = connection.ns =
    junctions.ns = confluence.ns = "myTopo";
  edges.ns = "parentEdges";
  edges2.ns = "childrenEdges";
  skeleton.ns = "skeleton";
  termination.action = faketermination.action = connection.action =
    junctions.action = confluence.action = edges.action = edges2.action =
      skeleton.action = visualization_msgs::Marker::ADD;
  termination.pose.orientation.w = faketermination.pose.orientation.w =
    connection.pose.orientation.w = junctions.pose.orientation.w =
      confluence.pose.orientation.w =
      edges.pose.orientation.w = edges2.pose.orientation.w =
        skeleton.pose.orientation.w = 1.0;

  termination.id = 0;
  faketermination.id = 1;
  connection.id = 2;
  junctions.id = 3;
  confluence.id = 7;
  edges.id = 4;
  edges2.id = 5;
  skeleton.id = 6;

  edges.type = edges2.type = visualization_msgs::Marker::LINE_LIST;
  termination.type = faketermination.type = connection.type = junctions.type = confluence.type =
    visualization_msgs::Marker::POINTS;
  skeleton.type = visualization_msgs::Marker::POINTS;

  edges.scale.x = 0.05;
  edges.scale.y = 0.05;
  edges.scale.z = 0.05;

  edges2.scale.x = 0.05;
  edges2.scale.y = 0.05;
  edges2.scale.z = 0.05;

  termination.scale.x = 0.1;
  termination.scale.y = 0.1;
  termination.scale.z = 0.1;

  faketermination.scale.x = 0.1;
  faketermination.scale.y = 0.1;
  faketermination.scale.z = 0.1;

  connection.scale.x = 0.1;
  connection.scale.y = 0.1;
  connection.scale.z = 0.1;

  junctions.scale.x = 0.2;
  junctions.scale.y = 0.2;
  junctions.scale.z = 0.2;

  confluence.scale.x = 0.2;
  confluence.scale.y = 0.2;
  confluence.scale.z = 0.2;

  skeleton.scale.x = 0.2;
  skeleton.scale.y = 0.2;
  skeleton.scale.z = 0.2;

  termination.color.r = 155.0f/255;
  termination.color.g = 89.0f/255;
  termination.color.b = 182.0f/255;

  faketermination.color.r = 52.0f/255;
  faketermination.color.g = 73.0f/255;
  faketermination.color.b = 94.0f / 255;

  connection.color.r = 44.0f / 255;
  connection.color.g = 232.0f / 255;
  connection.color.b = 123.0f / 255;

  junctions.color.r = 61.0f / 255;
  junctions.color.g = 177.0f / 255;
  junctions.color.b = 255.0f / 255;

  confluence.color.r = 255.0f / 255;
  confluence.color.g = 165.0f / 255;
  confluence.color.b = 2.0f / 255;

  skeleton.color.r = 231.0f / 255;
  skeleton.color.g = 76.0f / 255;
  skeleton.color.b = 60.0f / 255;

  edges.color.r = 214.0f / 255;
  edges.color.g = 48.0f / 255;
  edges.color.b = 49.0f / 255;

  edges2.color.r = 0.1f;
  edges2.color.g = 0.5f;
  edges2.color.b = 1.0f;

  termination.color.a = 1.0f;
  faketermination.color.a = 1.0f;
  connection.color.a = 1.0f;
  junctions.color.a = 1.0f;
  confluence.color.a = 1.0f;
  skeleton.color.a = 1.0f;
  edges.color.a = 0.3f;
  edges2.color.a = 0.5f;

  geometry_msgs::Point point;
  point.z = robot_bottom_;

  int num = 0;
  for (std::pair<VertexIter, VertexIter> vp = boost::vertices(heriTopo_);
       vp.first != vp.second; ++vp.first) {
    VertexDescriptor v = *vp.first;
    Point2D pt = index2Point(cvPoint2Index(heriTopo_.graph()[v].cvPt));
    Point2D pt2;
    point.x = pt.x();
    point.y = pt.y();

    point.z = 0.5;
    if (heriTopo_.graph()[v].removed) {
      skeleton.points.push_back(point);
    }

    if (heriTopo_.graph()[v].simplified_label > 0) {
      point.z = 1;
      confluence.points.push_back(point);
      for (auto &father_label :
           heriTopo_.graph()[v].unconfirmed_father_labels) {
        point.x = pt.x();
        point.y = pt.y();
        point.z = 1;
        edges2.points.push_back(point);
        pt2 = index2Point(nodeLabel2Index(father_label));
        point.x = pt2.x();
        point.y = pt2.y();
        point.z = 0.25;
        edges2.points.push_back(point);
           }
    }

    if (heriTopo_.graph()[v].simplified_label < -1) {
      point.z = -1;
      confluence.points.push_back(point);
    }

    pt = index2Point(cvPoint2Index(heriTopo_.graph()[v].cvPt));
    point.x = pt.x();
    point.y = pt.y();
    if (heriTopo_.graph()[v].confirmed)
      point.z = -robot_bottom_ * 3;
    else
      point.z = robot_bottom_;
    if (heriTopo_.graph()[v].type == Termination) {
      termination.points.push_back(point);
    } else if (heriTopo_.graph()[v].type == FakeTermination) {
      faketermination.points.push_back(point);
    } else if (heriTopo_.graph()[v].type == Connection) {
      connection.points.push_back(point);
    } else if (heriTopo_.graph()[v].type == Branch_junction) {
      junctions.points.push_back(point);
      for (auto &child_label : heriTopo_.graph()[v].children_labels) {
        point.x = pt.x();
        point.y = pt.y();
        edges2.points.push_back(point);
        pt2 = index2Point(nodeLabel2Index(child_label));
        point.x = pt2.x();
        point.y = pt2.y();
        edges2.points.push_back(point);
      }
    } else if (heriTopo_.graph()[v].type == Confluence_junction) {
      confluence.points.push_back(point);
      for (auto &father_label : heriTopo_.graph()[v].father_labels) {
        point.x = pt.x();
        point.y = pt.y();
        edges2.points.push_back(point);
        pt2 = index2Point(nodeLabel2Index(father_label));
        point.x = pt2.x();
        point.y = pt2.y();
        edges2.points.push_back(point);
      }
    }
    if (heriTopo_.graph()[v].type != Confluence_junction) {
      point.x = pt.x();
      point.y = pt.y();
      edges.points.push_back(point);
      pt2 = index2Point(nodeLabel2Index(heriTopo_.graph()[v].parent_label));
      point.x = pt2.x();
      point.y = pt2.y();
      edges.points.push_back(point);
    }

    num++;
       }
  ROS_INFO("my topo graph nodes num is %i", num);

  int number = 0;
  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(heriTopo_); ei != ei_end; ++ei) {
    number++;
  }
  ROS_INFO("the graph edges num is %i ", number);


  visualization_msgs::MarkerArray graph_markers;
  graph_markers.markers.resize(8);
  graph_markers.markers[0] = termination;
  graph_markers.markers[1] = faketermination;
  graph_markers.markers[2] = connection;
  graph_markers.markers[3] = junctions;
  graph_markers.markers[4] = edges;
  graph_markers.markers[5] = edges2;
  graph_markers.markers[6] = skeleton;
  graph_markers.markers[7] = confluence;

  return graph_markers;
}

void GlobalTopoManager::pubTopoMarkers() const {
  topo_pub_.publish(generateTopoMarkers());
}

std::vector<Point2D>
GlobalTopoManager::MYgetShortestPath(const Point2D &goal,
                                     const Point2D &start) const {
  std::vector<Eigen::Vector2d> path_points;
  path_points.clear();

  if (grid_map_.isInMapRange2D(start) && grid_map_.isInMapRange2D(goal)) {
    int start_grid_id =
        grid_map_.Index2DToGridId(grid_map_.getIndexInMap2D(start));
    int goal_grid_id =
        grid_map_.Index2DToGridId(grid_map_.getIndexInMap2D(goal));

    if (start_grid_id == goal_grid_id) {
      std::cout << "start point = end point, only return the start point or "
                   "end point as a path"
                << std::endl;
      path_points.clear();
      path_points.push_back(start);
      return path_points;
    }

    typedef std::pair<double, int> iPair;
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>>
        pq;
    std::vector<double> dist(grid_map_.grid_x_num_ * grid_map_.grid_y_num_,
                             INFINITY);
    std::vector<double> estimation(grid_map_.grid_x_num_ *
                                       grid_map_.grid_y_num_,
                                   INFINITY);
    const int INF = 0x3f3f3f3f;
    std::vector<int> backpointers(grid_map_.grid_x_num_ * grid_map_.grid_y_num_,
                                  INF);
    std::vector<bool> in_pq(grid_map_.grid_x_num_ * grid_map_.grid_y_num_,
                            false);

    dist[start_grid_id] = 0;
    estimation[start_grid_id] = (start - goal).norm();
    pq.push(std::make_pair(estimation[start_grid_id], start_grid_id));
    in_pq[start_grid_id] = true;

    std::cout << "A* search.." << std::endl;
    int u;
    int v;
    while (!pq.empty()) {
      u = pq.top().second;
      pq.pop();
      in_pq[u] = false;
      if (u == goal_grid_id) {
        break;
      }
      for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
          if (!(i == 0 && j == 0) && (u / grid_map_.grid_y_num_ + i) >= 0 &&
              (u / grid_map_.grid_y_num_ + i) < grid_map_.grid_x_num_ &&
              (u % grid_map_.grid_y_num_ + j) >= 0 &&
              (u % grid_map_.grid_y_num_ + j) < grid_map_.grid_y_num_) {
            v = (u / grid_map_.grid_y_num_ + i) * grid_map_.grid_y_num_ +
                (u % grid_map_.grid_y_num_ + j);

            if (grid_map_image_.at<uchar>(index2CvPoint(
                    Index2D{u / grid_map_.grid_y_num_ + i,
                            u % grid_map_.grid_y_num_ + j})) == 255) {
              if (dist[v] > dist[u] + grid_size_ * sqrt(i * i + j * j)) {
                dist[v] = dist[u] + grid_size_ * sqrt(i * i + j * j);
                estimation[v] =
                    dist[v] + (grid_map_.getGridCenter(v) -
                               grid_map_.getGridCenter(goal_grid_id))
                                  .norm();
                backpointers[v] = u;
                if (!in_pq[v]) {
                  pq.push(std::make_pair(estimation[v], v));
                  in_pq[v] = true;
                }
              }
            }
          }
        }
      }
    }

    std::vector<int> path;
    std::vector<int> reverse_path;
    int current = goal_grid_id;
    if (backpointers[current] == INF) {
      std::cout << "no path found" << std::endl;
      ROS_ERROR("no path found, return start and end node");
      path.clear();
      path_points.clear();
      path_points.push_back(start);
      path_points.push_back(goal);
    } else {
      while (current != INF) {
        reverse_path.push_back(current);
        current = backpointers[current];
      }

      path.clear();
      for (int i = reverse_path.size() - 1; i >= 0; --i) {
        path.push_back(reverse_path[i]);
      }

      path_points.clear();
      path_points.push_back(start);
      for (int i = 1; i < path.size() - 1; ++i) {
        path_points.push_back(grid_map_.getGridCenter(path[i]));
      }
      path_points.push_back(goal);
    }
  } else {
    std::cout << "start or end point is out of map range" << std::endl;
  }
  return path_points;
}


std::vector<Point2D> GlobalTopoManager::MYoptimalToStraight(std::vector<Point2D> &path) const {
  if (path.size() > 2) {
    std::vector<Point2D> pruned_path;
    std::vector<int> control_point_ids;
    int inner_idx = 0;
    int control_point_id = inner_idx;
    control_point_ids.push_back(control_point_id);
    while (inner_idx < path.size() - 1) {
      control_point_id = inner_idx;
      for (int i = inner_idx + 1; i < path.size(); ++i) {
        if (MYisCollisionFreeStraight(path[inner_idx], path[i])) {
          control_point_id = i;
        }
      }
      if (control_point_id == inner_idx) {
        control_point_id = inner_idx + 1;
      }
      control_point_ids.push_back(control_point_id);
      inner_idx = control_point_id;
    }

    for (int i = 0; i < control_point_ids.size(); ++i) {
      pruned_path.push_back(path[control_point_ids[i]]);
    }
    return pruned_path;
  } else {
    return path;
  }
}

visualization_msgs::MarkerArray
  GlobalTopoManager::generateOpencvCvMarkers(cv::Mat &for_vis, bool binary, double height) {

    visualization_msgs::Marker free;
    visualization_msgs::Marker occupied;
    visualization_msgs::Marker empty;
    visualization_msgs::Marker unknow;
    visualization_msgs::Marker center_point;

    empty.header.frame_id = occupied.header.frame_id = free.header.frame_id = unknow.header.frame_id = center_point.header.frame_id = "world";
    empty.header.stamp = occupied.header.stamp = free.header.stamp = unknow.header.stamp = center_point.header.stamp = ros::Time::now();
    occupied.ns = "occupied";
    free.ns = "free";
    empty.ns = "empty";
    unknow.ns = "unknown";
    center_point.ns = "center";
    empty.action = occupied.action = free.action = unknow.action = center_point.action = visualization_msgs::Marker::ADD;
    empty.pose.orientation.w = occupied.pose.orientation.w = free.pose.orientation.w = unknow.pose.orientation.w = center_point.pose.orientation.w = 1.0;

    free.id = 0;
    free.type = visualization_msgs::Marker::CUBE_LIST;

    occupied.id = 1;
    occupied.type = visualization_msgs::Marker::CUBE_LIST;

    empty.id = 2;
    empty.type = visualization_msgs::Marker::CUBE_LIST;

    unknow.id = 3;
    unknow.type = visualization_msgs::Marker::CUBE_LIST;

    center_point.id = 4;
    center_point.type = visualization_msgs::Marker::SPHERE_LIST;

    free.scale.x = grid_size_;
    free.scale.y = grid_size_;
    free.scale.z = 0.01;

    free.color.r = 0.5f;
    free.color.g = 1.0f;
    free.color.b = 0.5f;
    free.color.a = 0.5f;

    occupied.scale.x = grid_size_;
    occupied.scale.y = grid_size_;
    occupied.scale.z = 0.01;

    occupied.color.r = 0.1f;
    occupied.color.g = 0.5f;
    occupied.color.b = 0.5f;
    occupied.color.a = 1.0f;

    empty.scale.x = grid_size_;
    empty.scale.y = grid_size_;
    empty.scale.z = 0.01;

    empty.color.r = 0.5f;
    empty.color.g = 0.5f;
    empty.color.b = 0.5f;
    empty.color.a = 0.75f;

    unknow.scale.x = grid_size_;
    unknow.scale.y = grid_size_;
    unknow.scale.z = 0.01;

    unknow.color.r = 1.0f;
    unknow.color.g = 0.1f;
    unknow.color.b = 0.1f;
    unknow.color.a = 0.5f;

    center_point.scale.x = grid_size_;
    center_point.scale.y = grid_size_;
    center_point.scale.z = grid_size_;
    center_point.color.r = 1.0f;
    center_point.color.a = 1.0f;

    geometry_msgs::Point point;

  point.z = height;
    for (int i = 0; i < grid_num_x_; i++) {
      for (int j = 0; j < grid_num_y_; j++) {
        cv::Point cvPt = cv::Point(i, j);
        Point2D pt = index2Point(cvPoint2Index(cvPt));
        point.x = pt.x();
        point.y = pt.y();
        if (binary) {
          if (for_vis.at<uchar>(cvPt) == 255 or for_vis.at<uchar>(cvPt) == 1) {
            unknow.points.push_back(point);
          }
          else if (for_vis.at<uchar>(cvPt) == 0) {
            free.points.push_back(point);
          }
          else {
            ROS_ERROR("UNKNOWN VALUE = %i", for_vis.at<uchar>(cvPt));
          }
        }
        else {
          if (for_vis.at<uchar>(cvPt) == 255) {
            free.points.push_back(point);
          }
          else if (for_vis.at<uchar>(cvPt) == 0) {
            occupied.points.push_back(point);
          }
          else if (for_vis.at<uchar>(cvPt) == 64) {
            empty.points.push_back(point);
          }
          else if (for_vis.at<uchar>(cvPt) == 128) {
            unknow.points.push_back(point);
          }
        }
      }
    }

    visualization_msgs::MarkerArray grid_map_markers;
    grid_map_markers.markers.resize(5);
    grid_map_markers.markers[0] = free;
    grid_map_markers.markers[1] = occupied;
    grid_map_markers.markers[2] = empty;
    grid_map_markers.markers[3] = unknow;
    grid_map_markers.markers[4] = center_point;

    return grid_map_markers;
  }

}
