#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace hebi {
namespace ros {

class OdomPublisher {
public:
  OdomPublisher(rclcpp::Node& node);

  void send(const rclcpp::Time& time, const Eigen::Vector3d& global_pose_, const Eigen::Vector3d& global_vel_);

private:
  // Posts the calculated odometry to a topic
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  nav_msgs::msg::Odometry msg_;

  //Create a broadcaster for the TF for odom -> base_link
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_msgs::msg::TransformStamped tf_trans_;
};

} // namespace ros
} // namespace hebi
