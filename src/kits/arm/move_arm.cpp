#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <hebi_msgs/action/arm_motion.hpp>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif


namespace hebi {
namespace ros {

class MoveArm : public rclcpp::Node {
public:
  using ArmMotion = hebi_msgs::action::ArmMotion;
  using GoalHandleArmMotion = rclcpp_action::ClientGoalHandle<ArmMotion>;
  
  MoveArm() : Node("move_arm") {

    this->client_ptr_ = rclcpp_action::create_client<ArmMotion>(
      this,
      "arm_motion");
    
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MoveArm::send_waypoints, this));

    RCLCPP_INFO(this->get_logger(), "Initialized");

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

    auto goal_msg = ArmMotion::Goal();
    trajectory_msgs::msg::JointTrajectoryPoint point;

    RCLCPP_INFO(this->get_logger(), "Sending waypoints");
    point.positions = {0.24, -0.2, 0.27, M_PI, M_PI/2, M_PI/2};
    // point.positions = {0.01, 2.09439, 2.09439, 0.01, 1.5707963, 0.01};
    point.velocities = {0, 0, 0, 0, 0, 0};
    point.accelerations = {0, 0, 0, 0, 0, 0};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal_msg.waypoints.points.push_back(point);
    goal_msg.use_wp_times = true;
    goal_msg.wp_type = "CARTESIAN";

    int N = 50;
    for (int i = 1; i <= N; ++i) 
    {
      point.positions = {0.24, -0.2 + i*0.4/N, 0.27, M_PI, M_PI/2, M_PI/2};
      point.time_from_start = rclcpp::Duration::from_seconds(2 + ((double)i)/((double)N));
      goal_msg.waypoints.points.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Sending waypoints");

    auto send_goal_options = rclcpp_action::Client<ArmMotion>::SendGoalOptions();
    // send_goal_options.goal_response_callback = std::bind(&MoveArm::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveArm::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&MoveArm::result_callback, this, std::placeholders::_1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ArmMotion>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleArmMotion::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleArmMotion::SharedPtr, const std::shared_ptr<const ArmMotion::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Percent Complete: %f", feedback->percent_complete);
  }

  void result_callback(const GoalHandleArmMotion::WrappedResult & result) {
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
    RCLCPP_INFO(this->get_logger(), "Motion Completed");
    return;
  }
 
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::MoveArm>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}