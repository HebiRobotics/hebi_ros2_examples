#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

#include <hebi_msgs/action/base_motion.hpp>

#include "hebi_ros2_examples/diff_drive.hpp"
#include "hebi_ros2_examples/odom_publisher.hpp"


namespace hebi {
namespace ros {

class BaseNode : public rclcpp::Node {
public:
  using BaseMotion = hebi_msgs::action::BaseMotion;
  using GoalHandleBaseMotion = rclcpp_action::ServerGoalHandle<BaseMotion>;

  std::unique_ptr<hebi::DiffDrive> base_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr set_velocity_subscriber_;

  BaseNode() : Node("diff_drive_node") {

    this->declare_parameter("names", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("families", rclcpp::PARAMETER_STRING_ARRAY);

    if (!this->initializeBase()) {
      throw std::runtime_error("Aborting!");
    }

    Color c;
    base_->clearColor();

    // start the action server
    this->action_server_ = rclcpp_action::create_server<BaseMotion>(
      this,
      "base_motion",
      std::bind(&BaseNode::handleBaseMotionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BaseNode::handleBaseMotionCancel, this, std::placeholders::_1),
      std::bind(&BaseNode::handleBaseMotionAccepted, this, std::placeholders::_1));

    // Explicitly set the target velocity
    set_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&hebi::ros::BaseNode::updateVelocity, this, std::placeholders::_1));
  }

  //////////////////////// ACTION HANDLER FUNCTIONS ////////////////////////

  rclcpp_action::GoalResponse handleBaseMotionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BaseMotion::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received base motion action request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleBaseMotionCancel(const std::shared_ptr<GoalHandleBaseMotion> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel base motion action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleBaseMotionAccepted(const std::shared_ptr<GoalHandleBaseMotion> goal_handle) {
    // this needs to return quickly to avoid blocking the executor
    // spin up a new thread to execute the action
    std::thread{std::bind(&BaseNode::startBaseMotion, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Returns "false" if we have error...
  bool publishActionProgress(float start_prog, float end_prog, const std::shared_ptr<GoalHandleBaseMotion> goal_handle)
  {
    auto feedback = std::make_shared<BaseMotion::Feedback>();
    auto result = std::make_shared<BaseMotion::Result>();
    rclcpp::Rate r(10);

    while (!base_->isTrajectoryComplete(this->now().seconds()) && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        base_->clearColor();
        RCLCPP_INFO(this->get_logger(), "Base motion was cancelled");
        return false;
      }

      auto t = this->now().seconds();

      // Publish progress:
      auto& base_traj = base_->getTrajectory();
      feedback->percent_complete = start_prog + (end_prog - start_prog) * base_->trajectoryPercentComplete(t) / 2.0;
      goal_handle->publish_feedback(feedback);

      // Limit feedback rate
      r.sleep(); 
    }
    return true;
  }

  void startBaseMotion(const std::shared_ptr<GoalHandleBaseMotion> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing base motion action");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<BaseMotion::Result>();

    // Note: this is implemented right now as rotation, _THEN_ translation, _THEN_ rotation...
    // we can update this later.

    // Set color:
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    base_->setColor(color);

    // Face towards (x, y), assuming we start pointed with the x axis forward and the y axis left:
    bool success = true;
    if (goal->x != 0 || goal->y != 0)
    {
      base_->startRotateBy(std::atan2(goal->y, goal->x),
          this->now().seconds());
      success = publishActionProgress(0.0, 0.33, goal_handle);
      if (success) {
        base_->startMoveForward(std::sqrt(goal->x * goal->x + goal->y * goal->y),
          this->now().seconds());
        success = publishActionProgress(0.33, 0.67, goal_handle);
      }
      if (success) {
        base_->startRotateBy(-std::atan2(goal->y, goal->x) + goal->theta,
          this->now().seconds());
        success = publishActionProgress(0.67, 1.0, goal_handle);
      }
    } else {
      // Pure rotation:
      base_->startRotateBy(goal->theta,
          this->now().seconds());
      success = publishActionProgress(0.0, 1.0, goal_handle);
    }

    // Publish when the base is done with a motion
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      base_->clearColor();
      RCLCPP_INFO(this->get_logger(), "Completed base motion action");
    }
  }

  // Set the velocity, canceling any active action
  void updateVelocity(geometry_msgs::msg::Twist cmd_vel) {
    base_->startVelControl(cmd_vel.linear.x, cmd_vel.angular.z, this->now().seconds());
  }


private:

  rclcpp_action::Server<hebi_msgs::action::BaseMotion>::SharedPtr action_server_;

  /////////////////// Initialize base ///////////////////
  bool initializeBase() {
    
    // Get parameters for name/family of modules; default to standard values:
    std::vector<std::string> families;
    if (this->has_parameter("families")) {
      this->get_parameter("families", families);
      RCLCPP_INFO(this->get_logger(), "Found and successfully read 'families' parameter");
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'families' parameter; defaulting to 'DiffDrive'");
      families = {"DiffDrive"};
    }

    std::vector<std::string> names;
    if (this->has_parameter("names")) {
      this->get_parameter("names", names);
      RCLCPP_INFO(this->get_logger(), "Found and successfully read 'names' parameter");
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'names' parameter; defaulting to 'W1_left' and 'W2_right'");
      names = {"W1_left", "W2_right"};
    }

    // Create base and plan initial trajectory
    std::string error_out;

    std::string gains_file = ament_index_cpp::get_package_share_directory("hebi_description") + "/config/gains/diff_drive_gains.xml";

    std::string families_str = "", names_str = "";
    for (auto f : families) families_str += f + ", ";
    for (auto n : names) names_str += n + ", ";

    // print out the families, names, and gains file
    RCLCPP_INFO(this->get_logger(), "Families: %s", families_str.c_str());
    RCLCPP_INFO(this->get_logger(), "Names: %s", names_str.c_str());
    RCLCPP_INFO(this->get_logger(), "Gains file: %s", gains_file.c_str());

    for (int num_tries = 0; num_tries < 3; num_tries++) {
      base_ = hebi::DiffDrive::create(
        families, // Famil(ies)
        names, // Names
        gains_file, // Gains file
        this->now().seconds(), // Starting time (for trajectory)
        error_out);
      if (base_) {
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Could not initialize base, trying again...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!base_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to find the following modules in family: " << families.at(0));
      for(auto it = names.begin(); it != names.end(); ++it) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "> " << *it);
      }
      RCLCPP_ERROR_STREAM(this->get_logger(), error_out);
      RCLCPP_ERROR(this->get_logger(), "Could not initialize base! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
      return false;
    } else  {
      RCLCPP_INFO(this->get_logger(), "Base initialized!");
    }

    return true;
  }
};

} // namespace ros
} // namespace hebi


int main(int argc, char ** argv) {

  // Initialize ROS node
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<hebi::ros::BaseNode>();

    /////////////////// Main Loop ///////////////////
    while (rclcpp::ok()) {

      auto t = node->now();

      // Update feedback, and command the base to move along its planned path
      // (this also acts as a loop-rate limiter so no 'sleep' is needed)
      if (!node->base_->update(t.seconds()))
        RCLCPP_WARN(node->get_logger(), "Error Getting Feedback -- Check Connection");

      // Call any pending callbacks (note -- this may update our planned motion)
      rclcpp::spin_some(node);
    }
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("diff_drive_node"), "Caught runtime error: %s", e.what());
    return -1;
  }

  return 0;
}
