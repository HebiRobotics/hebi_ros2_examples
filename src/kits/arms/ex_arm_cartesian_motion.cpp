#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <hebi_msgs/msg/se3_trajectory.hpp>
#include <hebi_msgs/msg/se3_trajectory_point.hpp>
#include <hebi_msgs/action/arm_se3_motion.hpp>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif


namespace hebi {
namespace ros {

class MoveArmCartesian : public rclcpp::Node {
public:
  using ArmSE3Motion = hebi_msgs::action::ArmSE3Motion;
  using GoalHandleArmSE3Motion = rclcpp_action::ClientGoalHandle<ArmSE3Motion>;
  
  MoveArmCartesian() : Node("move_arm_cartesian") {

    this->client_ptr_ = rclcpp_action::create_client<ArmSE3Motion>(
      this,
      "cartesian_motion");

    // Pattern parameters
    this->y_min_ = -0.15;
    this->y_max_ = 0.15;
    this->z_min_ = 0.1;
    this->z_max_ = 0.3;
    this->num_points_ = 10;
    
    // Initial position
    this->start_position_ = {0.55, this->y_min_, this->z_min_, -M_PI/2, M_PI/2, 0.0};
    
    // Time parameters
    this->travel_time_ = 3.0;
    this->start_time_ = 2.0;
    
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MoveArmCartesian::send_waypoints, this));

    RCLCPP_INFO(this->get_logger(), "Initialized Cartesian Motion Example");
  }

  void send_waypoints() {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Action server ready");
    }

    auto goal_msg = ArmSE3Motion::Goal();
    
    // Set up the trajectory header
    goal_msg.waypoints.header.stamp = this->get_clock()->now();
    goal_msg.waypoints.header.frame_id = "base_link";
    goal_msg.use_wp_times = true;

    double t = this->start_time_;

    RCLCPP_INFO(this->get_logger(), "Sending cartesian waypoints for rectangle pattern");

    // Create start point
    hebi_msgs::msg::SE3TrajectoryPoint point;
    point.x = this->start_position_[0];
    point.y = this->start_position_[1];
    point.z = this->start_position_[2];
    point.roll = this->start_position_[3];
    point.pitch = this->start_position_[4];
    point.yaw = this->start_position_[5];
    point.gripper = 0.0;
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    goal_msg.waypoints.points.push_back(point);

    // Generate trajectory points for a rectangle pattern
    
    // 1. Horizontal movement from y_min to y_max
    for (int i = 0; i < this->num_points_; ++i) {
      double y = this->y_min_ + (this->y_max_ - this->y_min_) * (static_cast<double>(i) / (this->num_points_ - 1));
      point.x = 0.55;
      point.y = y;
      point.z = this->start_position_[2];
      point.roll = this->start_position_[3];
      point.pitch = this->start_position_[4];
      point.yaw = this->start_position_[5];
      point.gripper = 1.0;
      t += this->travel_time_ / this->num_points_;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      goal_msg.waypoints.points.push_back(point);
    }

    // 2. Vertical movement from z_min to z_max
    for (int i = 0; i < this->num_points_; ++i) {
      double z = this->z_min_ + (this->z_max_ - this->z_min_) * (static_cast<double>(i) / (this->num_points_ - 1));
      point.x = 0.55;
      point.y = this->y_max_;
      point.z = z;
      point.roll = this->start_position_[3];
      point.pitch = this->start_position_[4];
      point.yaw = this->start_position_[5];
      point.gripper = 0.0;
      t += this->travel_time_ / this->num_points_;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      goal_msg.waypoints.points.push_back(point);
    }

    // 3. Horizontal movement back from y_max to y_min
    for (int i = 0; i < this->num_points_; ++i) {
      double y = this->y_max_ - (this->y_max_ - this->y_min_) * (static_cast<double>(i) / (this->num_points_ - 1));
      point.x = 0.55;
      point.y = y;
      point.z = this->z_max_;
      point.roll = this->start_position_[3];
      point.pitch = this->start_position_[4];
      point.yaw = this->start_position_[5];
      point.gripper = 1.0;
      t += this->travel_time_ / this->num_points_;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      goal_msg.waypoints.points.push_back(point);
    }

    // 4. Vertical movement back from z_max to z_min
    for (int i = 0; i < this->num_points_; ++i) {
      double z = this->z_max_ - (this->z_max_ - this->z_min_) * (static_cast<double>(i) / (this->num_points_ - 1));
      point.x = 0.55;
      point.y = this->y_min_;
      point.z = z;
      point.roll = this->start_position_[3];
      point.pitch = this->start_position_[4];
      point.yaw = this->start_position_[5];
      point.gripper = 0.0;
      t += this->travel_time_ / this->num_points_;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      goal_msg.waypoints.points.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints", goal_msg.waypoints.points.size());

    auto send_goal_options = rclcpp_action::Client<ArmSE3Motion>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveArmCartesian::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveArmCartesian::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&MoveArmCartesian::result_callback, this, std::placeholders::_1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ArmSE3Motion>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Pattern parameters
  double y_min_, y_max_, z_min_, z_max_;
  int num_points_;
  std::vector<double> start_position_;
  double travel_time_, start_time_;

  void goal_response_callback(const GoalHandleArmSE3Motion::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleArmSE3Motion::SharedPtr, const std::shared_ptr<const ArmSE3Motion::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Percent Complete: %f", feedback->percent_complete);
  }

  void result_callback(const GoalHandleArmSE3Motion::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Cartesian Motion Completed");
    return;
  }
 
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::MoveArmCartesian>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
