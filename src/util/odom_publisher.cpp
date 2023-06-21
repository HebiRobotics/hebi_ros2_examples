#include "hebi_ros2_examples/odom_publisher.hpp"

#include "rclcpp/rclcpp.hpp"

namespace hebi {
namespace ros {

OdomPublisher::OdomPublisher(rclcpp::Node& node)
  : pub_(node.create_publisher<nav_msgs::msg::Odometry>("odom", 100)),
    broadcaster_(node) {
  // Parameter setup for NAV and Geometry messages
  // Odom and tf message setup; stamp and frame id assignment
  msg_.header.frame_id = "odom";
  msg_.child_frame_id = "base_footprint";

  tf_trans_.header.frame_id = "odom";
  tf_trans_.child_frame_id = "base_footprint";
}
  
void OdomPublisher::send(const rclcpp::Time& time, const Eigen::Vector3d& global_pose_, const Eigen::Vector3d& global_vel_) {
  // Timestamps
  msg_.header.stamp = time;
  tf_trans_.header.stamp = time;

  // Update messages
  tf2::Quaternion quat;
  quat.setRPY(0, 0, global_pose_[2]);

  // Position (odom pub)
  msg_.pose.pose.position.x = global_pose_.x();
  msg_.pose.pose.position.y = global_pose_.y();
  msg_.pose.pose.orientation.x = quat.x();
  msg_.pose.pose.orientation.y = quat.y();
  msg_.pose.pose.orientation.z = quat.z();
  msg_.pose.pose.orientation.w = quat.w();

  // Velocity (odom pub)
  msg_.twist.twist.linear.x = global_vel_.x();
  msg_.twist.twist.linear.y = global_vel_.y();
  msg_.twist.twist.angular.z = global_vel_.z();

  // Position (tf)
  tf_trans_.transform.translation.x = global_pose_.x();
  tf_trans_.transform.translation.y = global_pose_.y();
  tf_trans_.transform.rotation.x = quat.x();
  tf_trans_.transform.rotation.y = quat.y();
  tf_trans_.transform.rotation.z = quat.z();
  tf_trans_.transform.rotation.w = quat.w();

  // Publish odometry transform and message
  broadcaster_.sendTransform(tf_trans_);
  pub_->publish(msg_);
}

} // namespace ros
} // namespace hebi
