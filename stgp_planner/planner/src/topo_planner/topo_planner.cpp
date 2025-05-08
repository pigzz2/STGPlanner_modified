
#include "topo_planner/topo_planner.h"

namespace topo_planner {

    TopoPlanner::TopoPlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
            nh_(nh), nh_private_(nh_private), preprocess_inited_(false),
            planner_action_server_(nh_private_, "topo_planner",
                                   boost::bind(&TopoPlanner::plannerCallback, this, _1), false) {


        elements_ = std::make_shared<preprocess::Preprocess>(nh_, nh_private_);
        planner_ = std::make_shared<heri_graph_cover_planner::HeriGraphCoverPlanner>(nh_, nh_private_,
                                                                                    elements_->frontier_map_,
                                                                                    elements_->global_topo_map_);
        ros::Duration(0.5).sleep();
        ros::spinOnce();

        topo_planner_msgs_pub_ = nh_private_.advertise<control_planner_interface::PlannerMsgs>("topo_planner_msgs", 1);
        explore_finish_sub_ = nh_.subscribe<std_msgs::Bool>("exploration_data_finish", 1,
                                                            &TopoPlanner::explorationFinishCallback,
                                                            this);
        iteration_time_pub_ = nh_private_.advertise<visualization_tools::IterationTime>("iteration_time", 1);

        planner_action_server_.start();

        std::string pkg_path = ros::package::getPath("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";
        each_ufomap_update_txt_name_ = txt_path + "each_ufomap_updat_time.txt";
        each_topo_update_text_name_ = txt_path + "each_topo_update_time.txt";

        std::ofstream fout;

        sum_ufomap_update_time_ = 0;
        ufomap_update_num_ = 0;
        fout.open(each_ufomap_update_txt_name_, std::ios_base::in |
                                                    std::ios_base::out |
                                                    std::ios_base::trunc);
        fout << "each ufomap update time \n"
             << "start time \t"
             << "end time \t"
             << "elisped time \t"
             << "average time \t" << std::endl;
        fout.close();

        sum_topo_update_time_ = 0;
        topo_update_num_ = 0;
        fout.open(each_topo_update_text_name_, std::ios_base::in |
                                              std::ios_base::out |
                                              std::ios_base::trunc);
        fout << "each topo update time \n"
             << "start time \t"
             << "end time \t"
             << "elisped time \t"
             << "average time \t" << std::endl;
        fout.close();
    }

    void TopoPlanner::explorationFinishCallback(const std_msgs::BoolConstPtr &finish) {
        if (finish->data == true)
            ros::shutdown();
    }

    void
    TopoPlanner::plannerCallback(const control_planner_interface::ExplorerPlannerGoalConstPtr &goal) {
        elements_->global_topo_map_->global_topo_update_mutex_.lock();

        auto start_time = std::chrono::high_resolution_clock::now();
        double start_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                start_time.time_since_epoch()).count()) / 1000000;

        ros::WallTime ufomap_start_time = ros::WallTime::now();
        elements_->global_topo_map_->updateTopo();
        ros::WallTime topo_start_time = ros::WallTime::now();
        ros::WallTime update_end_time = ros::WallTime::now();
        planner_->Initialize(elements_->current_position_);
        preprocess_inited_ = true;

        control_planner_interface::ExplorerPlannerResult result;
        if (!preprocess_inited_) {
            ROS_WARN("preprocess not finish , waitting..");
            planner_action_server_.setSucceeded(result);
        } else if (planner_->tour_points_.empty() && planner_->no_frontier_left_) {
            ROS_WARN("tourpoints is empty, planning finish..");
            planner_action_server_.setSucceeded(result);
        } else {
            planner_goal_id_ = goal->iteration_id;
            ROS_INFO("start planning ...");
            bool is_successed = true;
            planner_->planning(elements_->current_pose_, elements_->forward_directory_,is_successed);
            if (is_successed) {
                if (planner_->tsp_path_.empty()) {
                    ROS_INFO("planner get a empty path");
                    result.paths.clear();
                } else {
                    ROS_INFO("planner way pose generate, result return");
                    for (int i = 0; i < planner_->path_segments_.size(); i++) {
                        control_planner_interface::Path path_segment;
                        path_segment.path = wayPoseGeneration(planner_->path_segments_[i]);
                        result.paths.push_back(path_segment);
                    }
                }
                planner_action_server_.setSucceeded(result);
                ROS_INFO("the iteration planning finish");
            } else {
                ROS_WARN("the iteration planning fail");
                planner_action_server_.setAborted(result);
            }
        }

        auto finish_time = std::chrono::high_resolution_clock::now();
        double finish_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                finish_time.time_since_epoch()).count()) / 1000000;

        double iteration_time = finish_time_second - start_time_second;

        visualization_tools::IterationTime time;
        time.iterationTime = iteration_time;
        time.current_time = finish_time_second;
        iteration_time_pub_.publish(time);

        ufomap_update_num_++;
        sum_ufomap_update_time_ = sum_ufomap_update_time_ + (topo_start_time - ufomap_start_time).toSec();
        std::ofstream fout;
        fout.open(each_ufomap_update_txt_name_,
                  std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << ufomap_start_time << "\t" << topo_start_time << "\t"
             << (topo_start_time - ufomap_start_time).toSec() << "\t" << ufomap_update_num_ << "\t"
             << sum_ufomap_update_time_ / ufomap_update_num_ << "s \t" << std::endl;
        fout.close();

        topo_update_num_++;
        sum_topo_update_time_ = sum_topo_update_time_ + (update_end_time - topo_start_time).toSec();
        fout.open(each_topo_update_text_name_,
                  std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << topo_start_time << "\t" << update_end_time << "\t"
             << (update_end_time - topo_start_time).toSec() << "\t" << topo_update_num_ << "\t"
             << sum_topo_update_time_ / topo_update_num_ << "s \t" << std::endl;
        fout.close();


        elements_->global_topo_map_->global_topo_update_mutex_.unlock();
    }

    std::vector<geometry_msgs::Pose> TopoPlanner::wayPoseGeneration(rapid_cover_planner::Path &path) {
        std::vector<geometry_msgs::Pose> way_poses;
        utils::Point3D current = elements_->current_position_;

        way_poses.clear();
        if (!path.empty()) {
            for (int i = 0; i < path.size(); i++) {
                tf::Vector3 point(path[i].x(), path[i].y(), path[i].z());

                tf::Vector3 axis(0, 0, 1);
                double angle;
                if (path.size() == 1) {
                    angle = std::atan2(path[i].y() - current.y(),
                                       path[i].x() - current.x());
                } else if ((i + 1) < path.size()) {
                    angle = std::atan2((path[i + 1].y() - path[i].y()),
                                       (path[i + 1].x() - path[i].x()));
                } else {
                    angle = std::atan2((path[i].y() - path[i - 1].y()),
                                       (path[i].x() - path[i - 1].x())); 
                }
                tf::Quaternion quaternion(axis, angle);
                tf::Pose poseTF(quaternion, point);
                geometry_msgs::Pose way_pose;
                tf::poseTFToMsg(poseTF, way_pose);

                way_poses.push_back(way_pose);
            }
        }
        return way_poses;
    }

    bool TopoPlanner::isExplorationFinish() {
        if (planner_->tour_points_.empty()) {
            return true;
        } else {
            return false;
        }
    }

    bool TopoPlanner::isCurrentGoalScanned() {
        int is_frontier_num = 0;
        for (const auto &frontier: planner_->goal_point_frontiers_) {
            if (elements_->frontier_map_->isFrontier(frontier)) {
                is_frontier_num++;
            }
        }
        if (is_frontier_num * elements_->frontier_map_->map_.getResolution() > planner_->viewpoint_ignore_thre_) {
            return false;
        } else {
            return true;
        }
    }

    void TopoPlanner::topo_planner_msgs_publish() {
        elements_->frontier_map_->map_mutex_.lock();
        elements_->map_2d_manager_->map_2d_update_mutex_.lock();
        elements_->elements_update_mutex_.lock();
        planner_->planner_mutex_.lock();

        control_planner_interface::PlannerMsgs msg;
        msg.current_iteration_id = planner_goal_id_;
        msg.current_goal_id = planner_goal_id_;
        msg.is_current_goal_scanned = isCurrentGoalScanned();
        msg.is_exploration_finished = isExplorationFinish();
        topo_planner_msgs_pub_.publish(msg);

        planner_->planner_mutex_.unlock();
        elements_->frontier_map_->map_mutex_.unlock();
        elements_->map_2d_manager_->map_2d_update_mutex_.unlock();
        elements_->elements_update_mutex_.unlock();
    }
}