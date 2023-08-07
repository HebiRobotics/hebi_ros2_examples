/**
 * Node to control Tready's base
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 19 Nov 2021
 */

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <hebi_msgs/msg/flipper_velocity_command.hpp>
#include <hebi_msgs/msg/treaded_base_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "hebi_ros2_examples/treaded_base.hpp"


namespace hebi {

class TreadedBaseNode : public rclcpp::Node {
public:
  std::unique_ptr<hebi::TreadedBase> base_;
  bool global_homing_{false};

  TreadedBaseNode() : Node("treaded_base_node") {
    this->declare_parameter("gains_package", rclcpp::PARAMETER_STRING);
    this->declare_parameter("gains_file", rclcpp::PARAMETER_STRING);

    if (!initializeBase()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Could not initialize base; aborting!");
      return;
    }

    base_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TreadedBaseNode::baseVelCallback, this, std::placeholders::_1));
    flipper_vel_subscriber_ = this->create_subscription<hebi_msgs::msg::FlipperVelocityCommand>("flipper_vel", 10, std::bind(&TreadedBaseNode::flipperVelCallback, this, std::placeholders::_1));
    color_subscriber_ = this->create_subscription<std_msgs::msg::ColorRGBA>("color", 10, std::bind(&TreadedBaseNode::colorCallback, this, std::placeholders::_1));

    state_publisher_ = this->create_publisher<hebi_msgs::msg::TreadedBaseState>("state", 100);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TreadedBaseNode::publishState, this));

    home_flippers_ = this->create_service<std_srvs::srv::SetBool>("home_flippers", std::bind(&TreadedBaseNode::homeService, this, std::placeholders::_1, std::placeholders::_2));
    align_flippers_ = this->create_service<std_srvs::srv::SetBool>("align_flippers", std::bind(&TreadedBaseNode::alignService, this, std::placeholders::_1, std::placeholders::_2));
    
  }
  
private:

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_vel_subscriber_;
  rclcpp::Subscription<hebi_msgs::msg::FlipperVelocityCommand>::SharedPtr flipper_vel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_subscriber_;

  // Publishers
  hebi_msgs::msg::TreadedBaseState state_msg_;
  rclcpp::Publisher<hebi_msgs::msg::TreadedBaseState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Service clients
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr home_flippers_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr align_flippers_;

  bool initializeBase() {
    // Read the package + path for the gains file
    std::string gains_package;
    if (this->has_parameter("gains_package")) {
      this->get_parameter("gains_package", gains_package);
      RCLCPP_INFO_STREAM(this->get_logger(), "Found and successfully read 'gains_package' parameter");
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find/read required 'gains_package' parameter; aborting!");
      return false;
    }
    std::string gains_file;
    if (this->has_parameter("gains_file")) {
      this->get_parameter("gains_file", gains_file);
      RCLCPP_INFO_STREAM(this->get_logger(), "Found and successfully read 'gains_file' parameter");
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find/read required 'gains_file' parameter; aborting!");
      return false;
    }

    std::string gains_path = ament_index_cpp::get_package_share_directory(gains_package) + std::string("/") + gains_file;

    hebi::Lookup lookup;
    std::string error;
    auto t_start = this->now().seconds();
    auto res = hebi::TreadedBase::create(lookup, "Tready", gains_path, t_start);
    base_ = std::move(std::get<0>(res));
    error = std::move(std::get<1>(res));
    if (!base_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), error.c_str());
      return false;
    }

    return true;
  }

  void baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Ignore input while aligning or homing flippers!
    if (base_->isAligning() || global_homing_)
      return;

    Eigen::VectorXd vels(3);
    vels << msg->linear.x, 0., msg->angular.z;
    auto t = this->now().seconds();
    base_->setChassisVelTrajectory(t, base_->chassisRampTime(), vels);
  }

  void flipperVelCallback(const hebi_msgs::msg::FlipperVelocityCommand::SharedPtr msg) {
    // Ignore input while aligning or homing flippers!
    if (base_->isAligning() || global_homing_)
      return;

    Eigen::VectorXd vels(4);
    vels[0] = msg->front_left;
    vels[1] = msg->front_right;
    vels[2] = msg->back_left;
    vels[3] = msg->back_right;
    auto t = this->now().seconds();
    base_->setFlipperTrajectory(t, base_->flipperRampTime(), nullptr, &vels);
  }

  void colorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg) {
    base_->setColor({(uint8_t)(msg->r * 255), (uint8_t)(msg->g * 255), (uint8_t)(msg->b * 255),
                        (uint8_t)(msg->a * 255)});
  }

  void publishState() {
    state_msg_.flippers_locked = base_->alignedFlipperMode();
    state_msg_.flippers_aligned = base_->flippersAligned();
    state_msg_.flipper_trajectory_active = base_->hasActiveFlipperTrajectory();
    state_msg_.base_trajectory_active = base_->hasActiveBaseTrajectory();
    state_msg_.mstop_pressed = base_->isMStopActive();
    state_publisher_->publish(state_msg_);
  }

  bool homeService(const std::shared_ptr<std_srvs::srv::SetBool::Request> flipper_command, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (global_homing_)
    {
      res->success = false;
      res->message = "Cannot home while homing!";
      return true;
    }
    global_homing_ = true;

    base_->clearChassisTrajectory();

    auto t = this->now().seconds();
    Eigen::VectorXd flipper_home(4);
    double tmp = 60. * M_PI / 180.; // 60 deg -> radians
    flipper_home << -tmp, tmp, tmp, -tmp;
    base_->setFlipperTrajectory(t, 5.0, &flipper_home, nullptr);

    res->success = true;

    return true;
  }

  bool alignService(const std::shared_ptr<std_srvs::srv::SetBool::Request> flipper_command, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (global_homing_)
    {
      res->success = false;
      res->message = "Cannot align while homing!";
      return true;
    }

    base_->setAlignedFlipperMode(flipper_command->data);

    res->success = true;

    return true;
  }
};
} // namespace hebi

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::TreadedBaseNode>();

  /////////////////// Main Loop ///////////////////
  while (rclcpp::ok()) {
    auto t = node->now().seconds();
    node->base_->update(t);
    if (!node->base_->hasActiveTrajectory())
      node->global_homing_ = false;
    node->base_->send();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}