#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

std::string source_frame_id;
std::string target_frame_id;

class PointCloudTransform {
public:
  PointCloudTransform()
      : tf_listener_(tf_buffer_) {

    const std::string &ns = ros::this_node::getName();
    source_frame_id = "sensor";
    if (!ros::param::get(ns + "/source_frame_id", source_frame_id)) {
      ROS_WARN("No source_frame_id specified. Looking for %s. Default is 'map'.",
               (ns + "/source_frame_id").c_str());
    }

    target_frame_id = "map";
    if (!ros::param::get(ns + "/target_frame_id", target_frame_id)) {
      ROS_WARN("No target_frame_id specified. Looking for %s. Default is 'sensor'.",
               (ns + "/target_frame_id").c_str());
    }

    pointcloud_sub_ = nh_.subscribe("/input_pointcloud", 10, &PointCloudTransform::pointcloudCallback, this);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/transformed_pointcloud", 10);
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    try {
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(1.0));

      sensor_msgs::PointCloud2 transformed_cloud;
      tf2::doTransform(*msg, transformed_cloud, transform_stamped);

      pointcloud_pub_.publish(transformed_cloud);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Transform warning: %s", ex.what());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher pointcloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_transform_node");
  PointCloudTransform pcl_transformer;
  ros::spin();
  return 0;
}
