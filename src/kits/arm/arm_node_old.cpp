#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <hebi_msgs/msg/target_waypoints.hpp>
#include <hebi_msgs/action/arm_motion.hpp>
#include <hebi_msgs/srv/set_ik_seed.hpp>

#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/arm/arm.hpp"


namespace arm = hebi::experimental::arm;

namespace hebi {
namespace ros {

class ArmNode : public rclcpp::Node {
public:
  ArmNode() : Node("arm_node") {

    this->declare_parameter("names", std::vector<std::string>({}));
    this->declare_parameter("families", std::vector<std::string>({}));
    this->declare_parameter("gains_package", "");
    this->declare_parameter("gains_file", "");
    this->declare_parameter("hrdf_package", "");
    this->declare_parameter("hrdf_file", "");
    this->declare_parameter("home_position", std::vector<double>({}));
    this->declare_parameter("moveit_joints", std::vector<std::string>({}));
    this->declare_parameter("ik_seed", std::vector<double>({}));

    if (!this->initializeArm()) {
      throw std::runtime_error("Aborting!");
    }

    if (this->has_parameter("ik_seed")) {
      std::vector<double> ik_seed;
      this->get_parameter("ik_seed", ik_seed);
      this->setIKSeed(ik_seed);
    } else {
      RCLCPP_WARN(this->get_logger(), "Param ik_seed not set, arm may exhibit erratic behavior");
    }

    // Subscribers
    offset_target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("offset_target", 50, std::bind(&ArmNode::offsetTargetCallback, this, std::placeholders::_1));
    set_target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("set_target", 50, std::bind(&ArmNode::setTargetCallback, this, std::placeholders::_1));
    joint_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_waypoints", 50, std::bind(&ArmNode::jointWaypointsCallback, this, std::placeholders::_1));
    cartesian_waypoint_subscriber_ = this->create_subscription<hebi_msgs::msg::TargetWaypoints>("cartesian_waypoints", 50, std::bind(&ArmNode::cartesianWaypointsCallback, this, std::placeholders::_1));

    // Publishers
    arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);
    center_of_mass_publisher_ = this->create_publisher<geometry_msgs::msg::Inertia>("inertia", 100);

    // Services
    compliant_mode_service_ = this->create_service<std_srvs::srv::SetBool>("compliance_mode", std::bind(&ArmNode::setCompliantMode, this, std::placeholders::_1, std::placeholders::_2));
    ik_seed_service_ = this->create_service<hebi_msgs::srv::SetIKSeed>("set_ik_seed", std::bind(&ArmNode::handleIKSeedService, this, std::placeholders::_1, std::placeholders::_2));
    
  }

  //////////////////////// SUBSCRIBER CALLBACK FUNCTIONS ////////////////////////

  // Callback for trajectories with joint angle waypoints
  void jointWaypointsCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory) {
    
    auto num_joints = arm_->size();
    auto num_waypoints = joint_trajectory->points.size();
    Eigen::MatrixXd pos(num_joints, num_waypoints);
    Eigen::MatrixXd vel(num_joints, num_waypoints);
    Eigen::MatrixXd accel(num_joints, num_waypoints);
    Eigen::VectorXd times(num_waypoints);

    for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint) {
      auto& cmd_waypoint = joint_trajectory->points[waypoint];

      if (cmd_waypoint.positions.size() != num_joints ||
          cmd_waypoint.velocities.size() != num_joints ||
          cmd_waypoint.accelerations.size() != num_joints) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Position, velocity, and acceleration sizes not correct for waypoint index " << waypoint);
        return;
      }

      if (!cmd_waypoint.effort.empty()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Effort commands in trajectories not supported; ignoring");
      }

      for (size_t joint = 0; joint < num_joints; ++joint) {
        pos(joint, waypoint) = cmd_waypoint.positions[joint];
        vel(joint, waypoint) = cmd_waypoint.velocities[joint];
        accel(joint, waypoint) = cmd_waypoint.accelerations[joint];
      }

      times(waypoint) = cmd_waypoint.time_from_start.sec;
    }
    updateJointWaypoints(pos, vel, accel, times);
  }

  // Callback for trajectories with cartesian position waypoints
  void cartesianWaypointsCallback(const hebi_msgs::msg::TargetWaypoints::SharedPtr target_waypoints) {

    // Fill in an Eigen::Matrix3xd with the xyz goal
    size_t num_waypoints = target_waypoints->waypoints_vector.size();
    Eigen::Matrix3Xd xyz_waypoints(3, num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i) {
      const auto& xyz_waypoint = target_waypoints->waypoints_vector[i];
      xyz_waypoints(0, i) = xyz_waypoint.x;
      xyz_waypoints(1, i) = xyz_waypoint.y;
      xyz_waypoints(2, i) = xyz_waypoint.z;
    }

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  // Set the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void setTargetCallback(const geometry_msgs::msg::Point::SharedPtr data) {

    // Fill in an Eigen::Matrix3Xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = data->x;
    xyz_waypoints(1, 0) = data->y;
    xyz_waypoints(2, 0) = data->z;

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  // "Jog" the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void offsetTargetCallback(const geometry_msgs::msg::Point::SharedPtr data) {

    // Only update if target changes!
    if (data->x == 0 && data->y == 0 && data->z == 0)
      return;

    // Initialize target from feedback as necessary
    if (!isTargetInitialized()) {
      auto pos = arm_->FK(arm_->lastFeedback().getPositionCommand());
      target_xyz_.x() = pos.x();
      target_xyz_.y() = pos.y();
      target_xyz_.z() = pos.z();
    }

    // Fill in an Eigen::Matrix3Xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = target_xyz_.x() + data->x;
    xyz_waypoints(1, 0) = target_xyz_.y() + data->y;
    xyz_waypoints(2, 0) = target_xyz_.z() + data->z;

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  //////////////////////// SERVICE HANDLER FUNCTIONS ////////////////////////
  bool setIKSeed(const std::vector<double>& ik_seed) {
    if(ik_seed.size() != home_position_.size()) {
      use_ik_seed_ = false;
    } else {
      use_ik_seed_ = true;
      ik_seed_.resize(ik_seed.size());
      for (size_t i = 0; i < ik_seed.size(); ++i) {
        ik_seed_[i] = ik_seed[i];
      }
    }
    return use_ik_seed_;
  }

  bool handleIKSeedService(const std::shared_ptr<hebi_msgs::srv::SetIKSeed::Request> req, std::shared_ptr<hebi_msgs::srv::SetIKSeed::Response> res) {
    return setIKSeed(req->seed);
  }

  bool setCompliantMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (req->data) {
      // Go into a passive mode so the system can be moved by hand
      res->message = "Pausing active command (entering grav comp mode)";
      arm_->cancelGoal();
      res->success = true;
    } else {
      res->message = "Resuming active command";
      // auto t = rclcpp::Node::now().seconds();
      auto last_position = arm_->lastFeedback().getPosition();
      arm_->setGoal(arm::Goal::createFromPosition(last_position));
      target_xyz_ = arm_->FK(last_position);
      res->success = true;
    }
    return true;
  }

  //////////////////////// OTHER UTILITY FUNCTIONS ////////////////////////

  void publishState() {
    auto& fdbk = arm_->lastFeedback();

    // how is there not a better way to do this?

    // Need to copy data from VectorXd to vector<double> in ros msg
    auto pos = fdbk.getPosition();
    auto vel = fdbk.getVelocity();
    auto eff = fdbk.getEffort();

    state_msg_.position.resize(pos.size());
    state_msg_.velocity.resize(vel.size());
    state_msg_.effort.resize(eff.size());
    state_msg_.header.stamp = this->now();

    Eigen::VectorXd::Map(&state_msg_.position[0], pos.size()) = pos;
    Eigen::VectorXd::Map(&state_msg_.velocity[0], vel.size()) = vel;
    Eigen::VectorXd::Map(&state_msg_.effort[0], eff.size()) = eff;

    arm_state_pub_->publish(state_msg_);

    // compute arm CoM
    auto& model = arm_->robotModel();
    Eigen::VectorXd masses;
    robot_model::Matrix4dVector frames;
    model.getMasses(masses);
    model.getFK(robot_model::FrameType::CenterOfMass, pos, frames);

    center_of_mass_message_.m = 0.0;
    Eigen::Vector3d weighted_sum_com = Eigen::Vector3d::Zero();
    for(int i = 0; i < model.getFrameCount(robot_model::FrameType::CenterOfMass); ++i) {
      center_of_mass_message_.m += masses(i);
      frames[i] *= masses(i);
      weighted_sum_com(0) += frames[i](0, 3);
      weighted_sum_com(1) += frames[i](1, 3);
      weighted_sum_com(2) += frames[i](2, 3);
    }
    weighted_sum_com /= center_of_mass_message_.m;

    center_of_mass_message_.com.x = weighted_sum_com(0);
    center_of_mass_message_.com.y = weighted_sum_com(1);
    center_of_mass_message_.com.z = weighted_sum_com(2);

    center_of_mass_publisher_->publish(center_of_mass_message_);
  }
  
  void setGoal() {
    this->arm_->setGoal(arm::Goal::createFromPosition(this->home_position_));
  }

  bool update() {
    return this->arm_->update();
  }

  bool send() {
    return this->arm_->send();
  }

private:
  std::unique_ptr<arm::Arm> arm_;

  // The end effector location that this arm will target (NaN indicates
  // unitialized state, and will be set from feedback during first
  // command)
  Eigen::Vector3d target_xyz_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  Eigen::VectorXd home_position_;

  Eigen::VectorXd ik_seed_{0};
  bool use_ik_seed_{false};

  sensor_msgs::msg::JointState state_msg_;
  geometry_msgs::msg::Inertia center_of_mass_message_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr offset_target_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_target_subscriber_;
  rclcpp::Subscription<hebi_msgs::msg::TargetWaypoints>::SharedPtr cartesian_waypoint_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_waypoint_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Inertia>::SharedPtr center_of_mass_publisher_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr compliant_mode_service_;
  rclcpp::Service<hebi_msgs::srv::SetIKSeed>::SharedPtr ik_seed_service_;
  
  // rclcpp_action::Server<hebi_msgs::action::ArmMotion>::SharedPtr action_server_;

  bool isTargetInitialized() {
    return !std::isnan(target_xyz_.x()) ||
           !std::isnan(target_xyz_.y()) ||
           !std::isnan(target_xyz_.z());
  }

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const Eigen::MatrixXd& angles, const Eigen::MatrixXd& velocities, const Eigen::MatrixXd& accelerations, const Eigen::VectorXd& times) {
    // Data sanity check:
    if (angles.rows() != velocities.rows()       || // Number of joints
        angles.rows() != accelerations.rows()    ||
        angles.rows() != arm_->size() ||
        angles.cols() != velocities.cols()       || // Number of waypoints
        angles.cols() != accelerations.cols()    ||
        angles.cols() != times.size()            ||
        angles.cols() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Angles, velocities, accelerations, or times were not the correct size");
      return;
    }

    // Update stored target position, based on final joint angles.
    target_xyz_ = arm_->FK(angles.rightCols<1>());

    // Replan:
    arm_->setGoal(arm::Goal::createFromWaypoints(times, angles, velocities, accelerations));
  }

  // Helper function to condense functionality between various message/action callbacks above
  // Replan a smooth joint trajectory from the current location through a
  // series of cartesian waypoints.
  // xyz positions should be a 3xn vector of target positions
  void updateCartesianWaypoints(const Eigen::Matrix3Xd& xyz_positions, const Eigen::Matrix3Xd* end_tip_directions = nullptr) {
    // Data sanity check:
    if (end_tip_directions && end_tip_directions->cols() != xyz_positions.cols())
      return;

    // Update stored target position:
    target_xyz_ = xyz_positions.col(xyz_positions.cols() - 1);

    // These are the joint angles that will be added
    auto num_waypoints = xyz_positions.cols();
    Eigen::MatrixXd positions(arm_->size(), num_waypoints);

    // Plan to each subsequent point from the last position
    // (We use the last position command for smoother motion)
    Eigen::VectorXd last_position = arm_->lastFeedback().getPositionCommand();

    if(use_ik_seed_) {
      last_position = ik_seed_;
    }

    // For each waypoint, find the joint angles to move to it, starting from the last
    // waypoint, and save into the position vector.
    if (end_tip_directions) {
      // If we are given tip directions, add these too...
      for (size_t i = 0; i < num_waypoints; ++i) {
        last_position = arm_->solveIK(last_position, xyz_positions.col(i), static_cast<Eigen::Vector3d>(end_tip_directions->col(i)));

        auto fk_check = arm_->FK(last_position);
        auto mag_diff = (last_position - fk_check).norm();
        if (mag_diff > 0.01) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Target Pose: "
                                                     << xyz_positions.col(i)[0] << ", "
                                                     << xyz_positions.col(i)[1] << ", "
                                                     << xyz_positions.col(i)[2]);
          RCLCPP_INFO_STREAM(this->get_logger(), "IK Solution: "
                                                    << last_position[0] << " | "
                                                    << last_position[1] << " | "
                                                    << last_position[2] << " | "
                                                    << last_position[3] << " | "
                                                    << last_position[4] << " | "
                                                    << last_position[5]);
          RCLCPP_WARN_STREAM(this->get_logger(), "Pose of IK Solution: "
                                                   << fk_check[0] << ", "
                                                   << fk_check[1] << ", "
                                                   << fk_check[2]);
          RCLCPP_INFO_STREAM(this->get_logger(), "Distance between target and IK pose: " << mag_diff);
        }
        positions.col(i) = last_position;
      }
    } else {
      for (size_t i = 0; i < num_waypoints; ++i) {
        last_position = arm_->solveIK(last_position, xyz_positions.col(i));
        positions.col(i) = last_position;
      }
    }

    // Replan:
    arm_->setGoal(arm::Goal::createFromPositions(positions));
  }

  /////////////////// Initialize arm ///////////////////
  bool initializeArm() {

    // Get parameters for name/family of modules; default to standard values:
    std::vector<std::string> families;
    if (this->has_parameter("families")) {
      this->get_parameter("families", families);
      RCLCPP_INFO(this->get_logger(), "Found and successfully read 'families' parameter");
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'families' parameter; defaulting to 'HEBI'");
      families = {"HEBI"};
    }

    std::vector<std::string> names;
    // Read the package + path for the gains file
    std::string gains_package;
    std::string gains_file;
    // Read the package + path for the hrdf file
    std::string hrdf_package;
    std::string hrdf_file;

    bool success = true;
    success = success && this->loadParam("names", names);
    success = success && this->loadParam("gains_package", gains_package);
    success = success && this->loadParam("gains_file", gains_file);
    success = success && this->loadParam("hrdf_package", hrdf_package);
    success = success && this->loadParam("hrdf_file", hrdf_file);

    if(!success) {
      RCLCPP_ERROR(this->get_logger(), "ABORTING!");
      return false;
    }

    // Create arm
    arm::Arm::Params params;
    params.families_ = families;
    params.names_ = names;
    // params.get_current_time_s_ = []() {
    //   static double start_time = this->now().seconds();
    //   return this->now().seconds() - start_time;
    // };

    params.hrdf_file_ = ament_index_cpp::get_package_share_directory(hrdf_package) + std::string("/") + hrdf_file;

    for (int num_tries = 0; num_tries < 3; num_tries++) {
      this->arm_ = arm::Arm::create(params);
      if (this->arm_) {
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Could not initialize arm, trying again...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!this->arm_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to find the following modules in family: " << families.at(0));
      for(auto it = names.begin(); it != names.end(); ++it) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "> " << *it);
      }
      RCLCPP_ERROR(this->get_logger(), "Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
      return false;
    } else  {
      RCLCPP_INFO(this->get_logger(), "Arm initialized!");
    }

    // Load the appropriate gains file
    if (!this->arm_->loadGains(ament_index_cpp::get_package_share_directory(gains_package) + std::string("/") + gains_file)) {
      RCLCPP_ERROR(this->get_logger(), "Could not load gains file and/or set arm gains. Attempting to continue.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Gains file loaded");
    }

    // Get the "home" position for the arm
    std::vector<double> home_position_vector;
    if (this->has_parameter("home_position")) {
      this->get_parameter("home_position", home_position_vector);
      RCLCPP_INFO(this->get_logger(), "Found and successfully read 'home_position' parameter");
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'home_position' parameter; defaulting to all zeros!");
    }
    this->home_position_ = Eigen::VectorXd(this->arm_->size());
    if (home_position_vector.empty()) {
      for (size_t i = 0; i < this->home_position_.size(); ++i) {
        this->home_position_[i] = 0.01; // Avoid common singularities by being slightly off from zero
      }
    } else if (home_position_vector.size() != this->arm_->size()) {
      RCLCPP_ERROR(this->get_logger(), "'home_position' parameter not the same length as HRDF file's number of DoF! Aborting!");
      return false;
    } else {
      for (size_t i = 0; i < this->home_position_.size(); ++i) {
        this->home_position_[i] = home_position_vector[i];
      }
    }

    // Make a list of family/actuator formatted names for the JointState publisher
    std::vector<std::string> full_names;
    for (size_t idx=0; idx<names.size(); ++idx) {
      full_names.push_back(families.at(0) + "/" + names.at(idx));
    }
    // TODO: Figure out a way to get link names from the arm, so it doesn't need to be input separately
    state_msg_.name = full_names;

    return true;
  }

  template <typename T>
  bool loadParam(std::string varname, T& var) {
    if (this->has_parameter(varname)) {
      if (this->get_parameter(varname, var)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Found and successfully read '" << varname << "' parameter");
        return true;
      }
    }

    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find/read required '" << varname << "' parameter!");
    return false;
  }
 
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<hebi::ros::ArmNode>();

    /////////////////// Main Loop ///////////////////

    // We update with a current timestamp so the "setGoal" function
    // is planning from the correct time for a smooth start

    auto t = node->now();

    node->update();
    node->setGoal();

    auto prev_t = t;
    while (rclcpp::ok()) {
      t = node->now();

      // Update feedback, and command the arm to move along its planned path
      // (this also acts as a loop-rate limiter so no 'sleep' is needed)
      if (!node->update())
        RCLCPP_WARN(node->get_logger(), "Error Getting Feedback -- Check Connection");
      else if (!node->send())
        RCLCPP_WARN(node->get_logger(), "Error Sending Commands -- Check Connection");

      node->publishState();

      // If a simulator reset has occurred, go back to the home position.
      if (t < prev_t) {
        RCLCPP_INFO(node->get_logger(), "Returning to home pose after simulation reset");
        node->setGoal();
      }
      prev_t = t;

      // Call any pending callbacks (note -- this may update our planned motion)
      rclcpp::spin_some(node);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_node"), "Caught runtime error: %s", e.what());
    return -1;
  }

  return 0;
}