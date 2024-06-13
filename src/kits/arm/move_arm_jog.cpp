#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/msg/joint_jog.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <hebi_msgs/action/arm_motion.hpp>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif


namespace hebi {
namespace ros {

class MoveArmJog : public rclcpp::Node {
public:
  
  MoveArmJog() : Node("move_arm_jog") {

        // Read parameters from robot's yaml
        this->declare_parameter("names", rclcpp::PARAMETER_STRING_ARRAY);
        this->declare_parameter("families", rclcpp::PARAMETER_STRING_ARRAY);
        this->declare_parameter("gains_package", rclcpp::PARAMETER_STRING);
        this->declare_parameter("gains_file", rclcpp::PARAMETER_STRING);
        this->declare_parameter("hrdf_package", rclcpp::PARAMETER_STRING);
        this->declare_parameter("hrdf_file", rclcpp::PARAMETER_STRING);
        this->declare_parameter("home_position", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("ik_seed", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("prefix", "");
        this->declare_parameter("use_traj_times", true);
        this->declare_parameter("compliant_mode", false);

        // Create Timer
        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/rate),
        std::bind(&MoveArmJog::timer_callback, this));

        // Publishers
        joint_jog_pub_ = this->create_publisher<control_msgs::msg::JointJog>("joint_jog", 50);
        cartesian_jog_pub_ = this->create_publisher<control_msgs::msg::JointJog>("cartesian_jog", 50);

        RCLCPP_INFO(this->get_logger(), "Initialized");

    }

    void timer_callback()
    {
        
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

    double nan = std::numeric_limits<double>::quiet_NaN();

    auto goal_msg = ArmMotion::Goal();
    trajectory_msgs::msg::JointTrajectoryPoint point;

    RCLCPP_INFO(this->get_logger(), "Sending waypoints");
    // point.positions = {0.24, -0.2, 0.27, M_PI, M_PI/2, M_PI/2};
    point.positions = {0.0, 2.09439, 2.09439, 0.0, M_PI/2};
    point.velocities = {0, 0, 0, 0, 0};
    point.accelerations = {0, 0, 0, 0, 0};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal_msg.waypoints.points.push_back(point);
    goal_msg.use_wp_times = true;
    goal_msg.wp_type = ArmMotion::Goal::JOINT_SPACE;

    double dt = 0.25, t = 2.0;
    while (t < 18.0)
    {
      t += dt;
      point.positions = {M_PI/2.0 * cos(M_PI*t/4.0), 2.09439, 2.09439, 0.0, M_PI/2};
      point.velocities = {-M_PI*M_PI/8.0 * sin(M_PI*t/4.0), 0.0, 0.0, 0.0, 0.0};
      point.accelerations = {nan, nan, nan, nan, nan};
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      goal_msg.waypoints.points.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Sending waypoints");

    auto send_goal_options = rclcpp_action::Client<ArmMotion>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveArm::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveArm::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&MoveArm::result_callback, this, std::placeholders::_1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  
    double rate = 200.0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr cartesian_jog_pub_;

    void goal_response_callback(const GoalHandleArmMotion::SharedPtr & goal_handle) {
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