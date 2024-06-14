#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hebi_msgs/action/arm_motion.hpp>

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
  
  ArmNode() : Node("arm_node") {

    // Parameter Description
    auto names_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto families_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto gains_package_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto gains_file_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto hrdf_package_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto hrdf_file_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto home_position_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto ik_seed_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto prefix_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_traj_times_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto compliant_mode_des = rcl_interfaces::msg::ParameterDescriptor{};

    names_des.description = "Array of Names of the HEBI Modules";
    families_des.description = "Array of Families of HEBI Modules";
    gains_package_des.description = "ROS package containing the gains file for your HEBI arm";
    gains_file_des.description = "Relative path of the gains file from the gains_package";
    hrdf_package_des.description = "ROS package containing the HRDF file for your HEBI arm";
    hrdf_file_des.description = "Relative path of the HRDF file from the hrdf_package";
    home_position_des.description = "Array of float values to move your arm after initialization.";
    ik_seed_des.description = "Seed for inverse kinematics. Can be changed during runtime.";
    prefix_des.description = "";
    use_traj_times_des.description = "";
    compliant_mode_des.description = "No arm motion can be commanded in compliant mode. Can be changed during runtime.";

    // Declare default parameter values
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

    // Parameter subscriber
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Parameter callbacks
    ik_seed_callback_handle_ = parameter_event_handler_->add_parameter_callback("ik_seed", std::bind(&ArmNode::ikSeedCallback, this, std::placeholders::_1));
    use_traj_times_callback_handle_ = parameter_event_handler_->add_parameter_callback("use_traj_times", std::bind(&ArmNode::useTrajTimesCallback, this, std::placeholders::_1));
    compliant_mode_callback_handle_ = parameter_event_handler_->add_parameter_callback("compliant_mode", std::bind(&ArmNode::compliantModeCallback, this, std::placeholders::_1));

    // Subscribers
    joint_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("joint_jog", 50, std::bind(&ArmNode::jointJogCallback, this, std::placeholders::_1));
    cartesian_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("cartesian_jog", 50, std::bind(&ArmNode::cartesianJogCallback, this, std::placeholders::_1));
    joint_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 50, std::bind(&ArmNode::jointWaypointsCallback, this, std::placeholders::_1));
    cartesian_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("cartesian_trajectory", 50, std::bind(&ArmNode::cartesianWaypointsCallback, this, std::placeholders::_1));

    // Publishers
    arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("fdbk_joint_states", 50);
    center_of_mass_publisher_ = this->create_publisher<geometry_msgs::msg::Inertia>("inertia", 50);
    end_effector_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 50);
    
    // start the action server
    action_server_ = rclcpp_action::create_server<ArmMotion>(
      this,
      "arm_motion",
      std::bind(&ArmNode::handleArmMotionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmNode::handleArmMotionCancel, this, std::placeholders::_1),
      std::bind(&ArmNode::handleArmMotionAccepted, this, std::placeholders::_1));

    // Initialize the arm
    if (!initializeArm()) {
      RCLCPP_ERROR(this->get_logger(), "Could not initialize arm!");
      return;
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmNode::publishState, this));

    // Go to home position
    try {
      arm_->update();
      arm_->setGoal(arm::Goal::createFromPosition(home_position_));
    }
    catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not go to home position: %s", e.what());
      return;
    }
  }

  void update() {
    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm_->update())
      RCLCPP_WARN(this->get_logger(), "Error Getting Feedback -- Check Connection");
    else if (!arm_->send())
      RCLCPP_WARN(this->get_logger(), "Error Sending Commands -- Check Connection");
  }

private:

  std::unique_ptr<arm::Arm> arm_;

  bool compliant_mode_{false};
  Eigen::VectorXd home_position_;
  int num_joints_;

  Eigen::VectorXd ik_seed_;
  bool use_ik_seed_{false};

  bool use_traj_times_{true};

  sensor_msgs::msg::JointState state_msg_;
  geometry_msgs::msg::Inertia center_of_mass_message_;

  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr cartesian_jog_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr cartesian_waypoint_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_waypoint_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Inertia>::SharedPtr center_of_mass_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::Server<hebi_msgs::action::ArmMotion>::SharedPtr action_server_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> ik_seed_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> use_traj_times_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> compliant_mode_callback_handle_;

  ////////////////////// PARAMETER CALLBACK FUNCTIONS //////////////////////
  void ikSeedCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(this->get_logger(), "Received an update to parameter 'ik_seed'");

    std::vector<double> ik_seed_vector;
    this->get_parameter("ik_seed", ik_seed_vector);
    if (ik_seed_vector.size() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter is empty; Ignoring!");
      use_ik_seed_ = false;
    }
    else if (ik_seed_vector.size() != this->arm_->size()) {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter not the same length as HRDF file's number of DoF! Ignoring!");
      use_ik_seed_ = false;
    }
    else 
    {
      ik_seed_ = Eigen::VectorXd(arm_->size());
      for (size_t i = 0; i < ik_seed_vector.size(); ++i) {
        ik_seed_[i] = ik_seed_vector[i];
      }
      RCLCPP_INFO(this->get_logger(), "Found and successfully updated 'ik_seed' parameter");
    }
  }

  void useTrajTimesCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type '%s': %s",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_bool() ? "true" : "false");

    this->get_parameter("use_traj_times", use_traj_times_);
    RCLCPP_INFO(this->get_logger(), "Found and successfully updated 'use_traj_times' parameter to %s", use_traj_times_ ? "true" : "false");
  }

  void compliantModeCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type '%s': %s",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_bool() ? "true" : "false");

    this->get_parameter("compliant_mode", compliant_mode_);
    RCLCPP_INFO(this->get_logger(), "Found and successfully updated 'compliant_mode' parameter to %s", compliant_mode_ ? "true" : "false");

    if (compliant_mode_) {
      setComplianceMode();
    } else {
      unsetComplianceMode();
    }
  }

  void setComplianceMode() {
    // Sleep for a bit to make sure the arm is done moving
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    // Set the arm to compliance mode
    arm_->cancelGoal();
    RCLCPP_INFO(this->get_logger(), "Arm is now in compliance mode!");
  }

  void unsetComplianceMode() {
    // Set a goal from current position to switch to action/topic mode
    arm_->setGoal(arm::Goal::createFromPosition(arm_->lastFeedback().getPosition()));
  }

  //////////////////////// ACTION HANDLER FUNCTIONS ////////////////////////

  rclcpp_action::GoalResponse handleArmMotionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ArmMotion::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received arm motion action request");
    (void)uuid;
    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - in compliant mode!");
      return rclcpp_action::GoalResponse::REJECT;
    }
    else if (goal->wp_type != ArmMotion::Goal::CARTESIAN_SPACE && goal->wp_type != ArmMotion::Goal::JOINT_SPACE) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - invalid waypoint type, should be 'CARTESIAN' or 'JOINT'");
      return rclcpp_action::GoalResponse::REJECT;
    }
    else if (goal->waypoints.points.empty()) {
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
    bool use_traj_times = goal->use_wp_times;

    int waypoint_type = goal->wp_type;

    if (waypoint_type == ArmMotion::Goal::CARTESIAN_SPACE) {
      // Get each waypoint in cartesian space
      Eigen::Matrix3Xd xyz_positions(3, num_waypoints);
      Eigen::Matrix3Xd orientation(3, num_waypoints);
      for (size_t i = 0; i < num_waypoints; ++i) {
        if (use_traj_times) {
          wp_times(i) = goal->waypoints.points[i].time_from_start.sec + goal->waypoints.points[i].time_from_start.nanosec * 1e-9;
        }
        xyz_positions(0, i) = goal->waypoints.points[i].positions[0];
        xyz_positions(1, i) = goal->waypoints.points[i].positions[1];
        xyz_positions(2, i) = goal->waypoints.points[i].positions[2];
        orientation(0, i) = goal->waypoints.points[i].positions[3];
        orientation(1, i) = goal->waypoints.points[i].positions[4];
        orientation(2, i) = goal->waypoints.points[i].positions[5];
      }

      updateCartesianWaypoints(use_traj_times, wp_times, xyz_positions, &orientation);
    } else if (waypoint_type == ArmMotion::Goal::JOINT_SPACE) {
      // Get each waypoint in joint space
      Eigen::MatrixXd pos(num_joints_, num_waypoints);
      Eigen::MatrixXd vel(num_joints_, num_waypoints);
      Eigen::MatrixXd accel(num_joints_, num_waypoints);

      for (size_t i = 0; i < num_waypoints; ++i) {
        if (use_traj_times) {
          wp_times(i) = goal->waypoints.points[i].time_from_start.sec + goal->waypoints.points[i].time_from_start.nanosec * 1e-9;
        }
        
        if (goal->waypoints.points[i].positions.size() != num_joints_ ||
            goal->waypoints.points[i].velocities.size() != num_joints_ ||
            goal->waypoints.points[i].accelerations.size() != num_joints_) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - Position, velocity, and acceleration sizes not correct for waypoint index");
          result->success = false;
          goal_handle->abort(result);
          return;
        }

        for (size_t j = 0; j < num_joints_; ++j) {
          pos(j, i) = goal->waypoints.points[i].positions[j];
          vel(j, i) = goal->waypoints.points[i].velocities[j];
          accel(j, i) = goal->waypoints.points[i].accelerations[j];
        }
      }

      updateJointWaypoints(use_traj_times, wp_times, pos, vel, accel);
    }

    // Set LEDs to a particular color, or clear them.
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    setColor(color);

    while (!arm_->atGoal() && rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        setColor({0, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Arm motion was cancelled");
        return;
      }

      // Check if compliant mode is active
      if (compliant_mode_) {
        result->success = false;
        goal_handle->abort(result);
        setColor({0, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Arm motion was aborted due to compliant mode activation");
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

  //////////////////////// SUBSCRIBER CALLBACK FUNCTIONS ////////////////////////

  // Callback for trajectories with joint angle waypoints
  void jointWaypointsCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory) {

    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Deactivate compliant mode to use joint trajectory");
      return;
    }

    if (joint_trajectory->points.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No waypoints specified");
      return;
    }

    // Print length of trajectory
    RCLCPP_INFO_STREAM(this->get_logger(), "Received trajectory with " << joint_trajectory->points.size() << " waypoints");

    auto num_waypoints = joint_trajectory->points.size();
    Eigen::MatrixXd pos(num_joints_, num_waypoints);
    Eigen::MatrixXd vel(num_joints_, num_waypoints);
    Eigen::MatrixXd accel(num_joints_, num_waypoints);
    Eigen::VectorXd times(num_waypoints);

    for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint) {
      auto& cmd_waypoint = joint_trajectory->points[waypoint];

      if (cmd_waypoint.positions.size() != num_joints_ ||
          cmd_waypoint.velocities.size() != num_joints_ ||
          cmd_waypoint.accelerations.size() != num_joints_) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Position, velocity, and acceleration sizes not correct for waypoint index " << waypoint);
        return;
      }

      if (!cmd_waypoint.effort.empty()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Effort commands in trajectories not supported; ignoring");
      }

      for (size_t joint = 0; joint < num_joints_; ++joint) {
        pos(joint, waypoint) = cmd_waypoint.positions[joint];
        vel(joint, waypoint) = cmd_waypoint.velocities[joint];
        accel(joint, waypoint) = cmd_waypoint.accelerations[joint];
      }

      times(waypoint) = cmd_waypoint.time_from_start.sec + cmd_waypoint.time_from_start.nanosec * 1e-9;
    }
    updateJointWaypoints(true, times, pos, vel, accel);
  }

  // Callback for trajectories with cartesian position waypoints
  void cartesianWaypointsCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr target_waypoints) {

    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Deactivate compliant mode to use cartesian trajectory");
      return;
    }

    if (target_waypoints->points.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No waypoints specified");
      return;
    }

    // Fill in an Eigen::Matrix3xd with the xyz goal
    size_t num_waypoints = target_waypoints->points.size();
    Eigen::Matrix3Xd xyz_positions(3, num_waypoints);
    Eigen::Matrix3Xd orientation(3, num_waypoints);
    Eigen::VectorXd times(num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i) {
      times(i) = target_waypoints->points[i].time_from_start.sec + target_waypoints->points[i].time_from_start.nanosec * 1e-9;
      xyz_positions(0, i) = target_waypoints->points[i].positions[0];
      xyz_positions(1, i) = target_waypoints->points[i].positions[1];
      xyz_positions(2, i) = target_waypoints->points[i].positions[2];
      orientation(0, i) = target_waypoints->points[i].positions[3];
      orientation(1, i) = target_waypoints->points[i].positions[4];
      orientation(2, i) = target_waypoints->points[i].positions[5];
    }

    // Replan
    updateCartesianWaypoints(use_traj_times_, times, xyz_positions, &orientation);
  }

  // "Jog" the arm along each joint
  void jointJogCallback(const control_msgs::msg::JointJog::SharedPtr jog_msg) {

    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Deactivate compliant mode to use joint jog");
      return;
    }

    bool inc_vel = true;
    if (jog_msg->velocities.empty()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "No velocities specified... Assuming zero velocity");
      inc_vel = false;
    } 
    else if (jog_msg->velocities.size() != num_joints_) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Velocities size not matching number of joints... Ignoring!");
      inc_vel = false;
    }

    // Get current joint position
    // (We use the last position command for smoother motion)
    auto cur_pos = arm_->lastFeedback().getPositionCommand();
    Eigen::MatrixXd pos(num_joints_, 1);
    Eigen::MatrixXd vel(num_joints_, 1);
    Eigen::MatrixXd accel(num_joints_, 1);
    Eigen::VectorXd times(1);

    if (jog_msg->displacements.size() != num_joints_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Displacement size not correct");
      return;
    }

    for (size_t joint = 0; joint < num_joints_; ++joint) {
      pos(joint, 0) = jog_msg->displacements[joint] + cur_pos[joint];
      if (inc_vel)
        vel(joint, 0) = jog_msg->velocities[joint];
      else
        vel(joint, 0) = 0.0;
      accel(joint, 0) = 0.0;
    }

    times(0) = jog_msg->duration;

    updateJointWaypoints(true, times, pos, vel, accel);
  }

  // "Jog" the target end effector location in cartesian space, replanning
  // smoothly to the new location
  void cartesianJogCallback(const control_msgs::msg::JointJog::SharedPtr jog_msg) {

    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Deactivate compliant mode to use cartesian jog");
      return;
    }

    if (jog_msg->displacements.size() != 3) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Displacement size not correct");
      return;
    }
    
    // Get current position and orientation
    // (We use the last position command for smoother motion)
    Eigen::Vector3d cur_pos;
    Eigen::Matrix3d cur_orientation;
    arm_->FK(arm_->lastFeedback().getPositionCommand(), cur_pos, cur_orientation);

    // Convert orientation to Euler angles
    Eigen::Vector3d cur_euler = cur_orientation.eulerAngles(0, 1, 2);

    Eigen::Matrix3Xd xyz_positions(3, 1);
    Eigen::Matrix3Xd orientation(3, 1);
    Eigen::VectorXd times(1);

    times(0) = jog_msg->duration;
    xyz_positions(0, 0) = cur_pos[0] + jog_msg->displacements[0];
    xyz_positions(1, 0) = cur_pos[1] + jog_msg->displacements[1];
    xyz_positions(2, 0) = cur_pos[2] + jog_msg->displacements[2];
    orientation(0, 0) = cur_euler[0];
    orientation(1, 0) = cur_euler[1];
    orientation(2, 0) = cur_euler[2];
    
    // Replan
    updateCartesianWaypoints(use_traj_times_, times, xyz_positions, &orientation);
  }

  /////////////////////////// UTILITY FUNCTIONS ///////////////////////////

  void publishState() {
    // Publish Joint State
    auto& fdbk = arm_->lastFeedback();

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

    // Publish End Effector Pose
    Eigen::Vector3d cur_pose;
    Eigen::Matrix3d cur_orientation;
    arm_->FK(pos, cur_pose, cur_orientation);
    Eigen::Quaterniond cur_orientation_quat(cur_orientation);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.pose.position.x = cur_pose[0];
    pose_msg.pose.position.y = cur_pose[1];
    pose_msg.pose.position.z = cur_pose[2];
    pose_msg.pose.orientation.x = cur_orientation_quat.x();
    pose_msg.pose.orientation.y = cur_orientation_quat.y();
    pose_msg.pose.orientation.z = cur_orientation_quat.z();
    pose_msg.pose.orientation.w = cur_orientation_quat.w();

    end_effector_pose_publisher_->publish(pose_msg);

    // Publish Center of Mass
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

  void setColor(const Color& color) {
    auto& command = arm_->pendingCommand();
    for (int i = 0; i < command.size(); ++i) {
      command[i].led().set(color);
    }
  }

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const bool use_traj_times, const Eigen::VectorXd& times, const Eigen::MatrixXd& angles, const Eigen::MatrixXd& velocities, const Eigen::MatrixXd& accelerations) {
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

    // Replan:
    if (use_traj_times) {
      arm_->setGoal(arm::Goal::createFromWaypoints(times, angles, velocities, accelerations));
    } else {
      arm_->setGoal(arm::Goal::createFromWaypoints(angles, velocities, accelerations));
    }
  }
  
  // Helper function to condense functionality between various message/action callbacks above
  // Replan a smooth joint trajectory from the current location through a
  // series of cartesian waypoints.
  // xyz positions should be a 3xn vector of target positions
  void updateCartesianWaypoints(const bool use_traj_times, const Eigen::VectorXd& times, const Eigen::Matrix3Xd& xyz_positions, const Eigen::Matrix3Xd* orientation = nullptr) {
    // Data sanity check:
    if (orientation && orientation->cols() != xyz_positions.cols())
      return;

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
    if (orientation && num_joints_ >= 6) {
      // If we are given tip directions, add these too...
      for (size_t i = 0; i < num_waypoints; ++i) {

        // Covert orientation to a 3x3 rotation matrix
        Eigen::Matrix3d rotation_matrix;

        // Roll -> Pitch -> Yaw ???
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
    if (use_traj_times)
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
    success = success && loadParam("names", names);
    success = success && loadParam("gains_package", gains_package);
    success = success && loadParam("gains_file", gains_file);
    success = success && loadParam("hrdf_package", hrdf_package);
    success = success && loadParam("hrdf_file", hrdf_file);

    // Print all parameters
    RCLCPP_INFO(this->get_logger(), "Successfully read the following parameters:");
    RCLCPP_INFO(this->get_logger(), "  families: %s", families[0].c_str());
    RCLCPP_INFO(this->get_logger(), "  names: %s", names[0].c_str());
    RCLCPP_INFO(this->get_logger(), "  gains_package: %s", gains_package.c_str());
    RCLCPP_INFO(this->get_logger(), "  gains_file: %s", gains_file.c_str());
    RCLCPP_INFO(this->get_logger(), "  hrdf_package: %s", hrdf_package.c_str());
    RCLCPP_INFO(this->get_logger(), "  hrdf_file: %s", hrdf_file.c_str());

    if(!success) {
      RCLCPP_ERROR(this->get_logger(), "ABORTING!");
      return false;
    }

    // Create arm
    arm::Arm::Params params;
    params.families_ = families;
    params.names_ = names;

    params.hrdf_file_ = ament_index_cpp::get_package_share_directory(hrdf_package) + std::string("/") + hrdf_file;

    for (int num_tries = 0; num_tries < 3; num_tries++) {
      arm_ = arm::Arm::create(params);
      if (arm_) {
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Could not initialize arm, trying again...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!arm_) {
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
    if (!arm_->loadGains(ament_index_cpp::get_package_share_directory(gains_package) + std::string("/") + gains_file)) {
      RCLCPP_WARN(this->get_logger(), "Could not load gains file and/or set arm gains. Attempting to continue.");
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
    home_position_ = Eigen::VectorXd(arm_->size());
    if (home_position_vector.empty()) {
      for (size_t i = 0; i < home_position_.size(); ++i) {
        home_position_[i] = 0.01; // Avoid common singularities by being slightly off from zero
      }
    } else if (home_position_vector.size() != arm_->size()) {
      RCLCPP_ERROR(this->get_logger(), "'home_position' parameter not the same length as HRDF file's number of DoF! Aborting!");
      return false;
    } else {
      for (size_t i = 0; i < home_position_.size(); ++i) {
        home_position_[i] = home_position_vector[i];
      }
    }

    // Get the "ik_seed" for the arm
    std::vector<double> ik_seed_vector;
    if (this->has_parameter("ik_seed")) {
      this->get_parameter("ik_seed", ik_seed_vector);
      if (ik_seed_vector.size() == 0)
      {
        RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter is empty; Setting use_ik_seed to false!");
        use_ik_seed_ = false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Found and successfully read 'ik_seed' parameter");
        if (ik_seed_vector.size() != this->arm_->size()) {
          RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter not the same length as HRDF file's number of DoF! Setting use_ik_seed to false!");
          use_ik_seed_ = false;
        }
        else 
        {
          ik_seed_ = Eigen::VectorXd(arm_->size());
          for (size_t i = 0; i < ik_seed_vector.size(); ++i) {
            ik_seed_[i] = ik_seed_vector[i];
          }
        }
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'ik_seed' parameter; Setting use_ik_seed to false!");
      use_ik_seed_ = false;
    }

    // Get the "use_traj_times" for the arm
    if (this->has_parameter("use_traj_times")) {
      this->get_parameter("use_traj_times", use_traj_times_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'use_traj_times' parameter; Setting use_traj_times to true!");
      use_traj_times_ = true;
    }

    // Get the "prefix" for the joint state topic
    std::string prefix;
    if (this->has_parameter("prefix")) {
      this->get_parameter("prefix", prefix);
    }

    // Make a list of family/actuator formatted names for the JointState publisher
    std::vector<std::string> full_names;
    for (size_t idx=0; idx<names.size(); ++idx) {
      full_names.push_back(prefix + names.at(idx));
    }
    // TODO: Figure out a way to get link names from the arm, so it doesn't need to be input separately
    state_msg_.name = full_names;
    num_joints_ = arm_->size();

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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::ArmNode>();

  while (rclcpp::ok()) {

    node->update();

    // Call any pending callbacks (note -- this may update our planned motion)
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}