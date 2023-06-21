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
  using ArmMotion = hebi_msgs::action::ArmMotion;
  using GoalHandleArmMotion = rclcpp_action::ServerGoalHandle<ArmMotion>;

  std::unique_ptr<arm::Arm> arm_;
  Eigen::VectorXd home_position_;
  
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

    // Publishers
    arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);
    center_of_mass_publisher_ = this->create_publisher<geometry_msgs::msg::Inertia>("inertia", 100);
    
    // start the action server
    this->action_server_ = rclcpp_action::create_server<ArmMotion>(
      this,
      "arm_motion",
      std::bind(&ArmNode::handleArmMotionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmNode::handleArmMotionCancel, this, std::placeholders::_1),
      std::bind(&ArmNode::handleArmMotionAccepted, this, std::placeholders::_1));
  }

  //////////////////////// ACTION HANDLER FUNCTIONS ////////////////////////

  rclcpp_action::GoalResponse handleArmMotionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ArmMotion::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received arm motion action request");
    (void)uuid;
    if (goal->wp_type != "cartesian" && goal->wp_type != "joint") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - invalid waypoint type, should be 'cartesian' or 'joint'");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->waypoints.points.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - no waypoints specified");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleArmMotionCancel(const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel arm motion action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleArmMotionAccepted(const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
    // this needs to return quickly to avoid blocking the executor
    // spin up a new thread to execute the action
    std::thread{std::bind(&ArmNode::startArmMotion, this, std::placeholders::_1), goal_handle}.detach();
  }

  void startArmMotion(const std::shared_ptr<GoalHandleArmMotion> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing arm motion action");
    
    // Wait until the action is complete, sending status/feedback along the way.
    rclcpp::Rate r(10);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ArmMotion::Feedback>();
    auto result = std::make_shared<ArmMotion::Result>();

    // Replan a smooth joint trajectory from the current location through a
    // series of cartesian waypoints.
    // TODO: use a single struct instead of 6 single vectors of the same length;
    // but how do we do hierarchial actions?
    auto num_waypoints = goal->waypoints.points.size();

    Eigen::VectorXd wp_times(num_waypoints);
    bool use_times = goal->use_wp_times;

    std::string waypoint_type = goal->wp_type;

    if (waypoint_type == "cartesian") {
      // Get each waypoint in cartesian space
      Eigen::Matrix3Xd xyz_positions(3, num_waypoints);
      Eigen::Matrix3Xd orientation(3, num_waypoints);
      for (size_t i = 0; i < num_waypoints; ++i) {
        if (use_times) {
          wp_times(i) = goal->waypoints.points[i].time_from_start.sec + goal->waypoints.points[i].time_from_start.nanosec * 1e-9;
        }
        xyz_positions(0, i) = goal->waypoints.points[i].positions[0];
        xyz_positions(1, i) = goal->waypoints.points[i].positions[1];
        xyz_positions(2, i) = goal->waypoints.points[i].positions[2];
        orientation(0, i) = goal->waypoints.points[i].positions[3];
        orientation(1, i) = goal->waypoints.points[i].positions[4];
        orientation(2, i) = goal->waypoints.points[i].positions[5];
      }

      updateCartesianWaypoints(use_times, wp_times, xyz_positions, &orientation);
    } else if (waypoint_type == "joint") {
      // Get each waypoint in joint space
      auto num_joints = this->arm_->size();
      Eigen::MatrixXd pos(num_joints, num_waypoints);
      Eigen::MatrixXd vel(num_joints, num_waypoints);
      Eigen::MatrixXd accel(num_joints, num_waypoints);

      for (size_t i = 0; i < num_waypoints; ++i) {
        if (use_times) {
          wp_times(i) = goal->waypoints.points[i].time_from_start.sec + goal->waypoints.points[i].time_from_start.nanosec * 1e-9;
        }
        
        if (goal->waypoints.points[i].positions.size() != num_joints ||
            goal->waypoints.points[i].velocities.size() != num_joints ||
            goal->waypoints.points[i].accelerations.size() != num_joints) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - Position, velocity, and acceleration sizes not correct for waypoint index");
          result->success = false;
          goal_handle->abort(result);
          return;
        }

        for (size_t j = 0; j < num_joints; ++j) {
          pos(j, i) = goal->waypoints.points[i].positions[j];
          vel(j, i) = goal->waypoints.points[i].velocities[j];
          accel(j, i) = goal->waypoints.points[i].accelerations[j];
        }
      }

      updateJointWaypoints(use_times, wp_times, pos, vel, accel);
    }

    // Set LEDs to a particular color, or clear them.
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    setColor(color);

    while (!arm_->atGoal() && rclcpp::ok()) {
      // check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        setColor({0, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Arm motion was cancelled");
        return;
      }

      // Update and publish progress in feedback
      feedback->percent_complete = arm_->goalProgress() * 100.0;
      goal_handle->publish_feedback(feedback);      

      // Limit feedback rate
      r.sleep();
    }

    // Publish when the arm is done with a motion
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Completed arm motion action");
      setColor({0, 0, 0, 0});
    }

  }

  //////////////////////// OTHER UTILITY FUNCTIONS ////////////////////////

  void setColor(const Color& color) {
    auto& command = arm_->pendingCommand();
    for (int i = 0; i < command.size(); ++i) {
      command[i].led().set(color);
    }
  }

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

private:
  

  // The end effector location that this arm will target (NaN indicates
  // unitialized state, and will be set from feedback during first
  // command)
  Eigen::Vector3d target_xyz_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  Eigen::VectorXd ik_seed_{0};
  bool use_ik_seed_{false};

  sensor_msgs::msg::JointState state_msg_;
  geometry_msgs::msg::Inertia center_of_mass_message_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Inertia>::SharedPtr center_of_mass_publisher_;

  rclcpp_action::Server<hebi_msgs::action::ArmMotion>::SharedPtr action_server_;

  bool isTargetInitialized() {
    return !std::isnan(target_xyz_.x()) ||
           !std::isnan(target_xyz_.y()) ||
           !std::isnan(target_xyz_.z());
  }

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const bool use_times, const Eigen::VectorXd& times, const Eigen::MatrixXd& angles, const Eigen::MatrixXd& velocities, const Eigen::MatrixXd& accelerations) {
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
    if (use_times) {
      arm_->setGoal(arm::Goal::createFromWaypoints(times, angles, velocities, accelerations));
    } else {
      arm_->setGoal(arm::Goal::createFromWaypoints(angles, velocities, accelerations));
    }
  }
  
  // Helper function to condense functionality between various message/action callbacks above
  // Replan a smooth joint trajectory from the current location through a
  // series of cartesian waypoints.
  // xyz positions should be a 3xn vector of target positions
  void updateCartesianWaypoints(const bool use_times, const Eigen::VectorXd& times, const Eigen::Matrix3Xd& xyz_positions, const Eigen::Matrix3Xd* orientation = nullptr) {
    // Data sanity check:
    if (orientation && orientation->cols() != xyz_positions.cols())
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

    auto current_position = arm_->FK(last_position);
    RCLCPP_INFO_STREAM(this->get_logger(), "Current Pose: "
                                             << current_position[0] << ", "
                                             << current_position[1] << ", "
                                             << current_position[2]);

    // For each waypoint, find the joint angles to move to it, starting from the last
    // waypoint, and save into the position vector.
    if (orientation) {
      // If we are given tip directions, add these too...
      for (size_t i = 0; i < num_waypoints; ++i) {

        RCLCPP_WARN_STREAM(this->get_logger(), "Target Pose: "
                                                     << xyz_positions.col(i)[0] << ", "
                                                     << xyz_positions.col(i)[1] << ", "
                                                     << xyz_positions.col(i)[2] << " ; "
                                                     << orientation->col(i)[0] << ", "
                                                     << orientation->col(i)[1] << ", "
                                                     << orientation->col(i)[2]) ;

        // Covert orientation to a 3x3 rotation matrix
        Eigen::Matrix3d rotation_matrix;

        rotation_matrix = Eigen::AngleAxisd(orientation->col(i)[0], Eigen::Vector3d::UnitX()).matrix()
                        * Eigen::AngleAxisd(orientation->col(i)[1], Eigen::Vector3d::UnitY()).matrix()
                        * Eigen::AngleAxisd(orientation->col(i)[2], Eigen::Vector3d::UnitZ()).matrix();

        last_position = arm_->solveIK(last_position, xyz_positions.col(i), rotation_matrix);

        auto fk_check = arm_->FK(last_position);
        auto mag_diff = (xyz_positions.col(i) - fk_check).norm();

        if (mag_diff > 0.01) {
          RCLCPP_WARN_STREAM(this->get_logger(), "IK Solution: "
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
          RCLCPP_WARN_STREAM(this->get_logger(), "Distance between target and IK pose: " << mag_diff);
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
    if (use_times)
      arm_->setGoal(arm::Goal::createFromPositions(times, positions));
    else
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

    node->arm_->update();
    node->arm_->setGoal(arm::Goal::createFromPosition(node->home_position_));

    auto prev_t = t;
    while (rclcpp::ok()) {
      t = node->now();

      // Update feedback, and command the arm to move along its planned path
      // (this also acts as a loop-rate limiter so no 'sleep' is needed)
      if (!node->arm_->update())
        RCLCPP_WARN(node->get_logger(), "Error Getting Feedback -- Check Connection");
      else if (!node->arm_->send())
        RCLCPP_WARN(node->get_logger(), "Error Sending Commands -- Check Connection");

      node->publishState();

      // If a simulator reset has occurred, go back to the home position.
      if (t < prev_t) {
        RCLCPP_INFO(node->get_logger(), "Returning to home pose after simulation reset");
        node->arm_->setGoal(arm::Goal::createFromPosition(node->home_position_));
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