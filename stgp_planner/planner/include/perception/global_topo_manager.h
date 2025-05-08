
#ifndef ROBO_PLANNER_WS_GLOBAL_TOPO_MANAGER_H
#define ROBO_PLANNER_WS_GLOBAL_TOPO_MANAGER_H

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

#include "grid_map_2d.h"
#include <traversability_analysis/TerrainMap.h>

#include "perception/ufomap.h"
#include "perception/hierarchical_graph.h"

namespace perception {


typedef ufo::map::OccupancyState UfoState;
typedef Status2D GridState;

class GlobalTopoManager {
public:
  typedef std::shared_ptr<GlobalTopoManager> Ptr;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  int time_cout_ = 0;
  int confluence_cluster_count_ = 0;
  std::string pkg_path = ros::package::getPath("planner");
  std::string txt_path = pkg_path + "/../../files/vis2/";


  ros::Timer global_topo_update_timer_;
  ros::Publisher global_topo_grid_pub_;
  ros::Publisher topo_pub_;
  ros::Publisher unknown_pub_;
  ros::Publisher grid_src_pub_;
  ros::Publisher skeleton_pub_;
  ros::Publisher full_skeleton_pub_;

  Ufomap::Ptr ufomap_;

  GridMap2D grid_map_;


  std::string each_grid_update_txt_name_;
  double sum_gridupdate_time_;
  int grid_update_num_;

  std::string each_skeleton_txt_name_;
  double sum_skeleton_time_;
  int skeleton_num_;

  double robot_height_;
  double robot_bottom_;
  double sensor_height_;

  double grid_size_;
  float min_x_;
  float min_y_;
  float max_x_;
  float max_y_;
  int grid_num_x_;
  int grid_num_y_;
  int trans_depth_;

  bool grid_changed_ = false;
  Index2D changed_grid_min_;
  Index2D changed_grid_max_;
  bool is_global_topo_grid_updated_ = false;
  Index2D robot_current_id_;
  geometry_msgs::Pose current_pose_;

  cv::Mat kernal_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat conn_kernal_ = (cv::Mat_<int>(3, 3) << 1, 1, 1, 1, 8, 1, 1, 1, 1);
  cv::Rect roi_;
  cv::Point roi_pt_;
  cv::Mat grid_map_image_;
  cv::Mat unconfirmed_skeleton_mask_;
  cv::Mat skeleton_mask_;
  cv::Mat unknown_mask_;
  cv::Mat wavefront_;
  cv::Mat removed_free_mask_;
  cv::Mat skeleton_mask_roi_;
  cv::Mat connection_num_roi_;
  bool incremental_thin_flag_ = false;

  bool initHome_ = false;
  bool initTopo_ = false;
  Index2D home_index2D_;
  int home_node_label_ = -1;
  std::vector<int> local_frontier_labels_;
  std::vector<int> global_frontier_labels_;
  std::vector<int> proc_confluence_labels_;
  std::vector<int> crop_termination_labels_;
  LabeledMap heriTopo_;

  struct ConfluenceCandidate {
    std::unordered_set<int> adjcents;
    std::vector<int> overlap_flag;
  };

  double inflate_radius_;
  double inflate_empty_radius_;
  double lower_z_;
  double connectivity_thre_;
  int wavefront_thresh_;
  int small_branch_len_upper_thresh_;
  int small_branch_len_lower_thresh_;
  int confluence_step_;

  std::mutex global_topo_update_mutex_;

  GlobalTopoManager(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private,
                    const Ufomap::Ptr &frontier_map);

  void getParamsFromRos();

  void init();

  void initGridMap();

  void initTopoMap();

  void timerCallback(const ros::TimerEvent &event);

  void updateGridMap();
  void updateSkeleton();
  void updateTopo();
  void cropSmallBranch();
  void removeBrance(int center_label);
  void processConfluence();

  void updateRobotPose(const double x, const double y, const double z) {
    robot_current_id_ = point2Index(Point2D{x,y});
    current_pose_.position.x = x;
    current_pose_.position.y = y;
    current_pose_.position.z = z;
  }

  void updateChangedGrid(const Index2D &grid_id);

  void resetChangedboundary() {
    changed_grid_max_ = {0, 0};
    changed_grid_min_ = {grid_num_x_, grid_num_y_};
  }

  void getChangedboundary();

  bool isInGridArea(const double &point_x, const double &point_y) const;

  bool isInGridArea(const Index2D &index) const;


  Point2D index2Point(const Index2D &index) const {
    return grid_map_.getGridCenter(index);
  }

  Index2D point2Index(const Point2D &point) const {
    return grid_map_.getIndexInMap2D(point);
  }

  static cv::Point index2CvPoint(const Index2D &index) {
    return {index.x(), index.y()};
  }

  Index2D cvPoint2Index(const cv::Point &point) const {
    return {point.x, point.y};
  }

  int index2NodeLabel(const Index2D &index) const {
    return index.y() * grid_num_x_ + index.x();
  }

  int cvPoint2NodeLabel(const cv::Point &pt) const {
    return pt.y * grid_num_x_ + pt.x;
  }

  cv::Point nodeLabel2CvPoint(const int &label) const
  {
    Index2D index = nodeLabel2Index(label);
    return index2CvPoint(index);
  }

  Index2D nodeLabel2Index(const int &label) const {
    return { label % grid_num_x_, label / grid_num_x_};
  }

  std::vector<Index2D> cell2Index(const Point2D &point, const int depth) const {
    std::vector<Index2D> indices;
    Index2D centerId = grid_map_.getIndexInMap2D(point);
    int centerId_x = centerId.x();
    int centerId_y = centerId.y();
    if (depth == 0) {
      indices.push_back(centerId);
    }
    else {
      int shift = pow(2, depth-1);
      for (int i = centerId_x - shift; i <= centerId_x + shift - 1; ++i) {
        for (int j = centerId_y - shift; j <= centerId_y + shift - 1; ++j) {
          if (i >= 0 && i <= grid_num_x_ && j >= 0 && j <=grid_num_y_)
          indices.push_back({i, j});
        }
      }
    }
    return indices;
  }

  void setMatState(const Index2D &index, const GridState &state) {
    is_global_topo_grid_updated_ = true;
    if (state == Free) {
      grid_map_image_.at<uchar>(index2CvPoint(index)) = 255;
    } else if (state == Occupied) {
      grid_map_image_.at<uchar>(index2CvPoint(index)) = 0;
    } else if (state == Unknown) {
      grid_map_image_.at<uchar>(index2CvPoint(index)) = 128;
    } else {
      grid_map_image_.at<uchar>(index2CvPoint(index)) = 64;
    }
  }


  void setHome(const Point2D &home) {
    home_index2D_ = point2Index(home);
    home_node_label_ = index2NodeLabel(home_index2D_);
  }

  void setHome(const Index2D &home) {
    home_index2D_ = home;
    home_node_label_ = index2NodeLabel(home_index2D_);
  }

  Index2D getHomeIndex() const { return home_index2D_; }

  int getHomeNodeLabel() const { return home_node_label_;}

  Node findNearestNode(double x, double y);

  std::vector<Index2D> get8Neighbors(const Index2D &center) const {
    std::vector<Index2D> invalid_nerghbors;
    static std::vector<Index2D> nerghbors {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (auto &n : nerghbors) {
      Index2D neighbor = center + n;
      if (isInGridArea(neighbor)) {
        invalid_nerghbors.push_back(neighbor);
      }
    }
    return invalid_nerghbors;
  }

  void updateLocalFrontier(int current_explore_branch);

  bool frontierClassify(int current_explore_branch, std::vector<MissedBranchMap> &history_missed_branches, bool &exist_missed_branches);

  void backtrackingForConfluenceJunction();

  void updateCurrentExploreBranch(int &current_explore_branch, std::vector<MissedBranchMap> &history_missed_branches, NodeQueue &passed_junctions);

  bool localFrontierRelocate(int &current_explore_branch, std::vector<MissedBranchMap> &history_missed_branches, NodeQueue &path);

  void updateNodesRecursion(const int &center_label, const cv::Point& center_cvPoint, int distance);

  void updateHistoryChildren(const int &center_label);

  void updateHistoryChildrenRec(int center_label, int parent_label, int parent_branch_label, int parent_junction_label, int distance);

  void getNeiborConfluence(int c_label, std::unordered_set<int> &candidates);

  void getNeiborConfluence(std::unordered_set<int> &c_labels, std::unordered_set<int> &candidates);

  bool haveOverlap(const std::unordered_set<int>& set1, const std::unordered_set<int>& set2);

  void updateNodeAsConfluence(int c_label, int f_label);

  void updateNodeAsConfluence(int c_label, std::vector<int> &f_labels);

  int updateFatherRec(int center_label, int parent_label, int parent_junction_label, int distance);

  void generateTopo();

  Node extendUnconfirmedFrontier(Node current_explore_branch, Node current_frontier, int lookadhead_thre);

  static int squaredIdDistance(const Index2D &id1, const Index2D &id2) {
    return (id1-id2).squaredNorm();
  }

  static double IdDistance(const Index2D &id1, const Index2D &id2) {
    return (id1-id2).norm();
  }

  bool isInRoi(const cv::Point &pt)const {
    return roi_.contains(pt);
  }

  std::vector<cv::Point> neighbor8_cvPt =
    {{-1, 0}, {1, 0}, {0, -1}, {0, 1}
    , {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

  visualization_msgs::MarkerArray generateTopoMarkers() const;

  void pubTopoMarkers() const;

  void getRoiForThin();

  bool incrementalThinning(const cv::Mat &binary, const cv::Mat &src, cv::Mat &confirmed_skeleton, cv::Mat& unknown,
                              cv::Mat &remove, cv::Mat &wavefront_count, cv::Mat &unconfirmed_skeleton, const bool check_unknown = false)const;

  void dilateSpecificValue(cv::Mat &image, int value) const;

  void initIncrementalMap();

  void writeImage();

  const std::vector<uint8_t> lut = {0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0,
                             1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0,
                             3, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             2, 0, 0, 0, 3, 0, 2, 2, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,
                             0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0,
                             3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0,
                             2, 0, 0, 0, 3, 1, 0, 0, 1, 3, 0, 0, 0, 0,
                             0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 1, 3, 1, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 1, 3,
                             0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                             0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      3, 3, 0, 1, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0};

  std::vector<Point2D> MYgetShortestPath(const Point2D &goal, const Point2D &start) const;

  std::vector<Point2D> MYoptimalToStraight(std::vector<Point2D> &path) const;

  inline bool MYisCollisionFreeStraight(const Point2D &source,
                                          const Point2D &target) const {
    if (grid_map_.isInMapRange2D(source) && grid_map_.isInMapRange2D(target)) {

      Index2D start_sub = grid_map_.getIndexInMap2D(source);
      Index2D end_sub = grid_map_.getIndexInMap2D(target);
      if (start_sub == end_sub)
        return true;
      Index2D diff_sub = end_sub - start_sub;
      double max_dist = diff_sub.squaredNorm();
      int step_x = grid_map_.signum(diff_sub.x());
      int step_y = grid_map_.signum(diff_sub.y());
      double t_max_x = step_x == 0
                           ? DBL_MAX
                           : grid_map_.intbound(start_sub.x(), diff_sub.x());
      double t_max_y = step_y == 0
                           ? DBL_MAX
                           : grid_map_.intbound(start_sub.y(), diff_sub.y());
      double t_delta_x =
          step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
      double t_delta_y =
          step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
      Index2D cur_sub = start_sub;

      while (grid_map_.isInMapRange2D(cur_sub) && cur_sub != end_sub &&
             (cur_sub - start_sub).squaredNorm() <= max_dist) {
        if (grid_map_image_.at<uchar>(index2CvPoint(cur_sub)) != 255) {
          return false;
        }
        if (t_max_x < t_max_y) {
          cur_sub.x() += step_x;
          t_max_x += t_delta_x;
        } else {
          cur_sub.y() += step_y;
          t_max_y += t_delta_y;
        }
             }
      if(!grid_map_.isInMapRange2D(cur_sub))
        return false;
      return true;
    }
  }

  visualization_msgs::MarkerArray
  generateOpencvCvMarkers(cv::Mat &for_vis, bool binary, double height);

};
}

#endif
