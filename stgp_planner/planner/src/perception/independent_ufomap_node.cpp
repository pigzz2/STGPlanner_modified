#include <ros/ros.h>

#include "perception/ufomap.h"

void timerCallback(const ros::TimerEvent& event, perception::Ufomap& ufomap) {
  ufomap.changed_cell_codes_.clear();
  for (auto it = ufomap.map_.changesBegin(); it != ufomap.map_.changesEnd(); it++) {
    ufomap.changed_cell_codes_.insert(*it);
  }
  ufomap.known_cell_codes_.insert(ufomap.changed_cell_codes_.begin(), ufomap.changed_cell_codes_.end());
}

int main(int argc, char** argv){
  ros::init(argc,argv,"independent_ufomap_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ROS_INFO_STREAM("start record ufomap");

  perception::Ufomap Ufomap(nh, nh_private);

  ros::Timer execution_timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(timerCallback, _1, boost::ref(Ufomap)));

  ros::spin();
  return 0;
}