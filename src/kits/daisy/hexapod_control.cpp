/**
 * Control a HEBI Hexapod (Lily) from ROS
 *
 * HEBI Robotics
 * September 2018
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "hebi_ros2_examples/leg.hpp"
#include "hebi_ros2_examples/hexapod.hpp"

#include "hebi_cpp_api/lookup.hpp"


namespace hebi {

// This is the ROS interface to the hexapod, receiving commands from messages
// and setting them on the hexapod object.
class HexapodNode : public rclcpp::Node {
public:
  std::unique_ptr<hebi::Hexapod> hexapod_;

  HexapodNode() : Node("hexapod_control") {

    // Publish joint state message  
    feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_feedback", 100);

    translation_velocity_cmd_.setZero();
    rotation_velocity_cmd_.setZero();

    // Controls to send to the robot
    angles = Eigen::VectorXd(num_joints_);
    vels = Eigen::VectorXd(num_joints_);
    torques = Eigen::VectorXd(num_joints_);
    foot_forces = Eigen::MatrixXd(3, 6);  // 3 (xyz) by num legs
    foot_forces.setZero();

    // Initialize hexapod
    if (!setupHexapod()) {
      RCLCPP_ERROR(this->get_logger(), "Could not setup hexapod!");
      return;
    }

    // Subscribe to velocity command and mode toggle messages
    vel_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "velocity_command", 50, std::bind(&HexapodNode::updateVelocityCommand, this, std::placeholders::_1));
    stance_mode_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "mode_select", 50, std::bind(&HexapodNode::updateMode, this, std::placeholders::_1));

    // Timer for publishing feedback
    feedback_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HexapodNode::publishFeedback, this));

    // Startup phase: smoothly transition!
    if (!startupHexapod()) {
      RCLCPP_ERROR(this->get_logger(), "Could not startup hexapod!");
      return;
    }
  }

  void updateVelocityCommand(geometry_msgs::msg::Twist vel_cmd) {
    // Limit the actual commanded velocities to reasonable max absolute values:
    constexpr double max_trans_vel = 0.175;
    constexpr double max_rot_vel = 0.4;
    translation_velocity_cmd_(0) = std::min(std::max(-vel_cmd.linear.x, -max_trans_vel), max_trans_vel);
    translation_velocity_cmd_(1) = std::min(std::max(-vel_cmd.linear.y, -max_trans_vel), max_trans_vel);
    translation_velocity_cmd_(2) = std::min(std::max(-vel_cmd.linear.z, -max_trans_vel), max_trans_vel);
    rotation_velocity_cmd_(0) = std::min(std::max(-vel_cmd.angular.x, -max_trans_vel), max_trans_vel);
    rotation_velocity_cmd_(1) = std::min(std::max(-vel_cmd.angular.y, -max_trans_vel), max_trans_vel);
    rotation_velocity_cmd_(2) = std::min(std::max(-vel_cmd.angular.z, -max_trans_vel), max_trans_vel);
  }

  void updateMode(std_msgs::msg::Bool stance_mode) {
    hexapod_->updateMode(stance_mode.data);
  }

  void publishFeedback() {

    Eigen::VectorXd positions = hexapod_->getLastPosition();
    Eigen::VectorXd velocities = hexapod_->getLastVelocity();
    Eigen::VectorXd efforts = hexapod_->getLastEffort();

    feedback_msg_.position.resize(positions.size());
    for (size_t i = 0; i < static_cast<size_t>(positions.size()); ++i)
      feedback_msg_.position[i] = positions[i];

    feedback_msg_.velocity.resize(velocities.size());
    for (size_t i = 0; i < static_cast<size_t>(velocities.size()); ++i)
      feedback_msg_.velocity[i] = velocities[i];

    feedback_msg_.effort.resize(efforts.size());
    for (size_t i = 0; i < static_cast<size_t>(efforts.size()); ++i)
      feedback_msg_.effort[i] = efforts[i];

    feedback_publisher_->publish(feedback_msg_);
  }

  void hexapodControlLoop() {

    cur_time_ = this->now().seconds();
    elapsed_time_ = cur_time_ - start_time_;
    dt_ = cur_time_ - prev_time_;
    prev_time_ = cur_time_;

    // Optionally slowly ramp up commands over the first few seconds
    double ramp_up_scale = std::min(1.0, elapsed_time_ / startup_seconds);

    // Actually control the hexapod:
    hexapod_->updateStance(
      translation_velocity_cmd_,
      rotation_velocity_cmd_,
      dt_);

    if (hexapod_->needToStep())
      hexapod_->startStep(elapsed_time_);

    hexapod_->updateSteps(elapsed_time_);

    // Calculate how the weight is distributed
    hexapod_->computeFootForces(elapsed_time_, foot_forces);

    foot_forces *= ramp_up_scale;

    Eigen::MatrixXd jacobian_ee;
    hebi::robot_model::MatrixXdVector jacobian_com;
    Eigen::VectorXd angles_plus_dt;

    for (int i = 0; i < 6; ++i) {
      hebi::Leg* curr_leg = hexapod_->getLeg(i);
      curr_leg->computeState(elapsed_time_, angles, vels, jacobian_ee, jacobian_com);

      // Get torques
      Eigen::Vector3d foot_force = foot_forces.block<3,1>(0,i);
      Eigen::Vector3d gravity_vec = hexapod_->getGravityDirection() * 9.8;
      torques = hexapod_->getLeg(i)->computeTorques(jacobian_com, jacobian_ee, angles, vels, gravity_vec, /*dynamic_comp_torque,*/ foot_force); // TODO:

      hexapod_->setCommand(i, &angles, &vels, &torques);
    }
    hexapod_->sendCommand();

  }

private: 

  int num_joints_{hebi::Leg::getNumJoints()};

  Eigen::Vector3d translation_velocity_cmd_;
  Eigen::Vector3d rotation_velocity_cmd_;

  // publish feedback!
  sensor_msgs::msg::JointState feedback_msg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_publisher_;

  // Timer for publishing feedback
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stance_mode_subscriber_;

  // Timekeeping variables
  double start_time_;
  double dt_{0};
  double prev_time_;
  double cur_time_;
  double elapsed_time_;
  double startup_seconds{4.5};

  // Controls to send to the robot
  Eigen::VectorXd angles;
  Eigen::VectorXd vels;
  Eigen::VectorXd torques;
  Eigen::MatrixXd foot_forces;

  /////////////////// Initialize hexapod ///////////////////
  bool setupHexapod() {
    std::string resource_path = ament_index_cpp::get_package_share_directory("hebi_description") + "/config/gains/hexapod/";

    hebi::HexapodErrors hex_errors;
    hebi::HexapodParameters params;
    if (!params.loadFromFile(resource_path + "/hex_config.xml")) {
      RCLCPP_ERROR(this->get_logger(), "Could not find parameter file at %s", (resource_path + "/hex_config.xml").c_str());
      return false;
    }
    params.resource_path_ = resource_path;

    hexapod_ = hebi::Hexapod::create(params, hex_errors);

    for (int num_tries = 0; num_tries < 3; num_tries++) {
      hexapod_ = hebi::Hexapod::create(params, hex_errors);
      if (hexapod_) {
        RCLCPP_INFO(this->get_logger(), "Found robot! Starting control program...");
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Could not initialize hexapod, trying again...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!hexapod_) {
      RCLCPP_ERROR(this->get_logger(), "Could not initialize hexapod! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
      return false;
    }

    // Load gains file
    if (!hexapod_->setGains()) {
      RCLCPP_WARN(this->get_logger(), "Unable to set hexapod gains, things may behave strangely");
    }

    start_time_ = this->now().seconds();

    return true;
  }

  bool initializeHexapod(std::vector<std::shared_ptr<hebi::trajectory::Trajectory>> &startup_trajectories) {
    try {
      cur_time_ = this->now().seconds();
      elapsed_time_ = cur_time_ - start_time_;

      // V l in legs, let s_l = start joints and e_l = end joints and t_l =
      // trajectory, with zero vel/accel endpoints.
      for (int i = 0; i < 6; ++i)
      {
        Eigen::VectorXd leg_start = hexapod_->getLegFeedback(i);
        Eigen::VectorXd leg_end;
        Eigen::VectorXd leg_vels;
        Eigen::MatrixXd jacobian_ee(0, 0);
        hebi::robot_model::MatrixXdVector jacobian_com;
        hexapod_->getLeg(i)->computeState(elapsed_time_, leg_end, leg_vels, jacobian_ee, jacobian_com);
        // TODO: fix! (quick and dirty -- leg mid is hardcoded as offset from leg end)
        Eigen::VectorXd leg_mid = leg_end;
        leg_mid(1) -= 0.3;
        leg_mid(2) -= 0.15;

        // Convert for trajectories
        int num_waypoints = 5;
        Eigen::MatrixXd positions(num_joints_, num_waypoints);
        Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_joints_, num_waypoints);
        Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_joints_, num_waypoints);
        Eigen::VectorXd nan_column = Eigen::VectorXd::Constant(num_joints_, std::numeric_limits<double>::quiet_NaN());
        // Is this one of the legs that takes a step first?
        bool step_first = (i == 0 || i == 3 || i == 4);

        // Set positions
        positions.col(0) = leg_start;
        positions.col(1) = step_first ? leg_mid : leg_start;
        positions.col(2) = step_first ? leg_end : leg_start;
        positions.col(3) = step_first ? leg_end : leg_mid;
        positions.col(4) = leg_end;

        velocities.col(1) = nan_column;
        velocities.col(3) = nan_column;
        accelerations.col(1) = nan_column;
        accelerations.col(3) = nan_column;

        Eigen::VectorXd times(num_waypoints);
        double local_start = elapsed_time_;
        double total = startup_seconds - local_start;
        times << local_start,
                  local_start + total * 0.25,
                  local_start + total * 0.5,
                  local_start + total * 0.75,
                  local_start + total;
        startup_trajectories.push_back(hebi::trajectory::Trajectory::createUnconstrainedQp(
          times, positions, &velocities, &accelerations));
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception thrown while initializing hexapod: %s", e.what());
      return false;
    }
    return true;
  }

  bool startupHexapod() {
    std::vector<std::shared_ptr<hebi::trajectory::Trajectory>> startup_trajectories;
    if (!initializeHexapod(startup_trajectories)) {
      return false;
    }

    rclcpp::Rate r(100);

    while (rclcpp::ok() && elapsed_time_ < startup_seconds) {
      cur_time_ = this->now().seconds();
      elapsed_time_ = cur_time_ - start_time_;
      dt_ = cur_time_ - prev_time_;
      prev_time_ = cur_time_;

      // Follow t_l:
      for (int i = 0; i < 6; ++i)
      {
        Eigen::VectorXd v(3);
        Eigen::VectorXd a(3);
        startup_trajectories[i]->getState(elapsed_time_, &angles, &v, &a);
        vels = v;

        Eigen::Vector3d foot_force = foot_forces.block<3,1>(0,i);
        hebi::Leg* curr_leg = hexapod_->getLeg(i);

        // Get the Jacobian
        Eigen::MatrixXd jacobian_ee;
        hebi::robot_model::MatrixXdVector jacobian_com;
        curr_leg->computeJacobians(angles, jacobian_ee, jacobian_com);

        Eigen::Vector3d gravity_vec = hexapod_->getGravityDirection() * 9.8;
        torques = curr_leg->computeTorques(jacobian_com, jacobian_ee, angles, vels, gravity_vec, /* dynamic_comp_torque,*/ foot_force); // TODO: add dynamic compensation
        // TODO: add actual foot torque for startup?
        // TODO: add vel, torque; test each one!
        hexapod_->setCommand(i, &angles, &vels, &torques);
      }
      hexapod_->sendCommand();

      r.sleep();
    }

    return true;
  }
};

} // namespace hebi

int main(int argc, char** argv) {

  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::HexapodNode>();

  // Run at 100 Hz:
  rclcpp::Rate rate(100);
  // Main command loop
  while (rclcpp::ok()) {

    node->hexapodControlLoop();

    // Call any pending callbacks (note -- this may update our planned motion)
    rclcpp::spin_some(node);

    rate.sleep();
  }

  return 0;
}
