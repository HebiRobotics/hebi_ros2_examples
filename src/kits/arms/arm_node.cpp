#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hebi_msgs/action/arm_motion.hpp>

#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/robot_config.hpp"
#include "hebi_cpp_api/arm/arm.hpp"


namespace arm = hebi::experimental::arm;

namespace hebi {
namespace ros {

class ArmNode : public rclcpp::Node {
public:
  using ArmMotion = hebi_msgs::action::ArmMotion;
  using GoalHandleArmMotion = rclcpp_action::ServerGoalHandle<ArmMotion>;
  
  ArmNode() : Node("arm_node") {

    // Parameter Descriptions
    // Parameters passed into the node
    auto config_file_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto config_package_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto prefix_des = rcl_interfaces::msg::ParameterDescriptor{};

    // Parameters passed through config file changed during runtime
    auto ik_seed_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_traj_times_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto compliant_mode_des = rcl_interfaces::msg::ParameterDescriptor{};

    config_file_des.description = "Config file for the arm of type .cfg.yaml.";
    config_package_des.description = "Package containg the config file.";
    prefix_des.description = "Prefix for the arm.";
    ik_seed_des.description = "Seed for inverse kinematics. Can be changed during runtime.";
    use_traj_times_des.description = "Can be changed during runtime.";
    compliant_mode_des.description = "No arm motion can be commanded in compliant mode. Can be changed during runtime.";

    // Declare default parameter values
    this->declare_parameter("config_file", rclcpp::PARAMETER_STRING);
    this->declare_parameter("config_package", rclcpp::PARAMETER_STRING);
    this->declare_parameter("prefix", "");
    this->declare_parameter("ik_seed", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("use_traj_times", true);
    this->declare_parameter("compliant_mode", false);

    // Get the parameters that are passed into the node
    config_package_ = this->get_parameter("config_package").as_string();
    config_file_ = this->get_parameter("config_file").as_string();

    // Initialize the arm with configs
    if (!initializeArm()) {
      RCLCPP_ERROR(this->get_logger(), "Could not initialize arm! Please check if the modules are available on the network.");
      return;
    }

    // Event handler for parameter changes
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Parameter callbacks
    ik_seed_callback_handle_ = parameter_event_handler_->add_parameter_callback("ik_seed", std::bind(&ArmNode::ikSeedCallback, this, std::placeholders::_1));
    use_traj_times_callback_handle_ = parameter_event_handler_->add_parameter_callback("use_traj_times", std::bind(&ArmNode::useTrajTimesCallback, this, std::placeholders::_1));
    compliant_mode_callback_handle_ = parameter_event_handler_->add_parameter_callback("compliant_mode", std::bind(&ArmNode::compliantModeCallback, this, std::placeholders::_1));

    // Subscribers
    joint_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("joint_jog", 10, std::bind(&ArmNode::jointJogCallback, this, std::placeholders::_1));
    cartesian_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("cartesian_jog", 10, std::bind(&ArmNode::cartesianJogCallback, this, std::placeholders::_1));
    SE3_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("SE3_jog", 10, std::bind(&ArmNode::SE3JogCallback, this, std::placeholders::_1));
    joint_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10, std::bind(&ArmNode::jointWaypointsCallback, this, std::placeholders::_1));
    cartesian_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("cartesian_trajectory", 10, std::bind(&ArmNode::cartesianWaypointsCallback, this, std::placeholders::_1));
    if (num_joints_ == 6) {
      cmd_ee_wrench_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>("cmd_ee_wrench", 10, std::bind(&ArmNode::wrenchCommandCallback, this, std::placeholders::_1));
    }

    // Publishers
    arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    center_of_mass_publisher_ = this->create_publisher<geometry_msgs::msg::Inertia>("inertia", 10);
    end_effector_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
    if (num_joints_ == 6) {
      ee_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ee_wrench", 10);
    }
    else RCLCPP_WARN(this->get_logger(), "Cannot publish wrench data for this arm, as it does not have 6 joints");

    // Services
    home_service_ = this->create_service<std_srvs::srv::Trigger>("home", std::bind(&ArmNode::homeCallback, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<std_srvs::srv::Trigger>("stop", std::bind(&ArmNode::stopCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Start the action server
    action_server_ = rclcpp_action::create_server<ArmMotion>(
      this,
      "arm_motion",
      std::bind(&ArmNode::handleArmMotionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmNode::handleArmMotionCancel, this, std::placeholders::_1),
      std::bind(&ArmNode::handleArmMotionAccepted, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&ArmNode::publishState, this));

    // Go to home position if available
    if (home_position_available_)
      std::thread{std::bind(&ArmNode::homeArm, this)}.detach();
    else stopArm();   // Essentially we are just setting the arm to stay at current position

    arm_initialized_ = true;
  }

  void update() {
    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm_->update())
      RCLCPP_WARN(this->get_logger(), "Error Getting Feedback -- Check Connection");
    else {
      // Modify pending command here
      arm_->pendingCommand().setEffort(arm_->pendingCommand().getEffort() + cmd_joint_effort_);
      // Send command
      if (!arm_->send())
        RCLCPP_WARN(this->get_logger(), "Error Sending Commands -- Check Connection");
    }
  }

private:

  std::string config_package_ = "";
  std::string config_file_ = "";

  std::unique_ptr<arm::Arm> arm_;

  bool arm_initialized_{false};
  bool has_active_action_{false};
  bool is_homing_{false};

  bool compliant_mode_{false};
  Eigen::VectorXd home_position_{ Eigen::VectorXd::Constant(6, 0.01) }; // Default values are close to zero to avoid singularity
  bool home_position_available_{false};
  
  Eigen::VectorXd cmd_joint_effort_{ Eigen::VectorXd::Zero(6) };
  
  int num_joints_;

  Eigen::VectorXd ik_seed_;
  bool use_ik_seed_{false};

  bool use_traj_times_{true};

  sensor_msgs::msg::JointState state_msg_;
  geometry_msgs::msg::Inertia center_of_mass_message_;

  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr cartesian_jog_subscriber_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr SE3_jog_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr cartesian_waypoint_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_waypoint_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr cmd_ee_wrench_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Inertia>::SharedPtr center_of_mass_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ee_wrench_publisher_;

  rclcpp_action::Server<hebi_msgs::action::ArmMotion>::SharedPtr action_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  rclcpp::TimerBase::SharedPtr timer_;

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
    if (!arm_initialized_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - arm not initialized!");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - in compliant mode!");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (has_active_action_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - arm already has an active trajectory!");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (is_homing_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - arm is currently homing!");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->wp_type != ArmMotion::Goal::CARTESIAN_SPACE && goal->wp_type != ArmMotion::Goal::JOINT_SPACE) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - invalid waypoint type, should be 'CARTESIAN' or 'JOINT'");
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
    
    // Set active action flag
    has_active_action_ = true;

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
      Eigen::Matrix3Xd euler_angles(3, num_waypoints);
      for (size_t i = 0; i < num_waypoints; ++i) {
        if (use_traj_times) {
          wp_times(i) = goal->waypoints.points[i].time_from_start.sec + goal->waypoints.points[i].time_from_start.nanosec * 1e-9;
        }
        xyz_positions(0, i) = goal->waypoints.points[i].positions[0];
        xyz_positions(1, i) = goal->waypoints.points[i].positions[1];
        xyz_positions(2, i) = goal->waypoints.points[i].positions[2];
        euler_angles(0, i) = goal->waypoints.points[i].positions[3];
        euler_angles(1, i) = goal->waypoints.points[i].positions[4];
        euler_angles(2, i) = goal->waypoints.points[i].positions[5];
      }

      updateSE3Waypoints(use_traj_times, wp_times, xyz_positions, &euler_angles, false);
    } else if (waypoint_type == ArmMotion::Goal::JOINT_SPACE) {
      // Get each waypoint in joint space
      Eigen::MatrixXd pos(num_joints_, num_waypoints);
      Eigen::MatrixXd vel(num_joints_, num_waypoints);
      Eigen::MatrixXd accel(num_joints_, num_waypoints);

      for (size_t i = 0; i < num_waypoints; ++i) {
        if (use_traj_times) {
          wp_times(i) = goal->waypoints.points[i].time_from_start.sec + goal->waypoints.points[i].time_from_start.nanosec * 1e-9;
        }
        
        if (goal->waypoints.points[i].positions.size() != static_cast<size_t>(num_joints_) ||
            goal->waypoints.points[i].velocities.size() != static_cast<size_t>(num_joints_) ||
            goal->waypoints.points[i].accelerations.size() != static_cast<size_t>(num_joints_)) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - Position, velocity, and acceleration sizes not correct for waypoint index");
          result->success = false;
          goal_handle->abort(result);

          // Reset active action flag
          has_active_action_ = false;
          return;
        }

        for (size_t j = 0; j < static_cast<size_t>(num_joints_); ++j) {
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
        stopArm();
        result->success = false;
        goal_handle->canceled(result);
        setColor({0, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Arm motion was cancelled");
        
        // Reset active action flag
        has_active_action_ = false;
        return;
      }

      // Check if compliant mode is active
      if (compliant_mode_) {
        result->success = false;
        goal_handle->abort(result);
        setColor({0, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Arm motion was aborted due to compliant mode activation");
        
        // Reset active action flag
        has_active_action_ = false;
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
      // Reset active action flag
      has_active_action_ = false;
    }

  }

  ///////////////////////////// SERVICE CALLBACKS /////////////////////////////

  // Service callback for homing the arm
  void homeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    response->success = true;

    if (!arm_initialized_) {
      response->success = false;
      response->message = "Arm not initialized yet!";
    }

    if (!home_position_available_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Home position was not set in the config file!");
      return;
    }

    if (compliant_mode_) {
      response->success = false;
      response->message = "Arm in compliant mode, cannot home arm";
    }

    if (has_active_action_) {
      response->success = false;
      response->message = "Arm is executing an action, cannot home arm";
    }

    if (is_homing_) {
      response->success = false;
      response->message = "Arm is currently homing";
    }

    if (!response->success) {
      RCLCPP_ERROR_STREAM(this->get_logger(), response->message);
      return;
    }

    response->message = "Homing requested";
    
    // Start separate thread to home the arm
    std::thread{std::bind(&ArmNode::homeArm, this)}.detach();
  }

  // Service callback for stopping the arm
  void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    response->success = true;

    if (!arm_initialized_) {
      response->success = false;
      response->message = "Arm not initialized yet!";
    }

    if (compliant_mode_) {
      response->success = false;
      response->message = "Arm in compliant mode, nothing to stop";
    }

    if (has_active_action_) {
      response->success = false;
      response->message = "Cannot stop using /stop service, cancel the active action to stop arm motion";
    }

    if (!response->success) {
      RCLCPP_ERROR_STREAM(this->get_logger(), response->message);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Stopping arm motion");
    stopArm();
    response->message = "Stopped arm motion";
  }

  //////////////////////// SUBSCRIBER CALLBACK FUNCTIONS ////////////////////////

  // Common subscriber condition checks for the arm
  bool checkArmConditions(std::string topic_name) {
    if (!arm_initialized_) {
      RCLCPP_ERROR(this->get_logger(), "Arm is not initialized yet!");
      return false;
    }

    if (compliant_mode_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Arm in compliant mode, ignoring " << topic_name);
      return false;
    }

    if (has_active_action_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Arm is executing an action, ignoring " << topic_name);
      return false;
    }

    if (is_homing_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Arm is currently homing, ignoring " << topic_name);
      return false;
    }

    return true;
  }

  // Callback for trajectories with joint angle waypoints
  void jointWaypointsCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory) {

    if (!checkArmConditions("joint_trajectory"))
      return;

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

      if (cmd_waypoint.positions.size() != static_cast<size_t>(num_joints_) ||
          cmd_waypoint.velocities.size() != static_cast<size_t>(num_joints_) ||
          cmd_waypoint.accelerations.size() != static_cast<size_t>(num_joints_)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Position, velocity, and acceleration sizes not correct for waypoint index " << waypoint);
        return;
      }

      if (!cmd_waypoint.effort.empty()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Effort commands in trajectories not supported; ignoring");
      }

      for (size_t joint = 0; joint < static_cast<size_t>(num_joints_); ++joint) {
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

    if (!checkArmConditions("cartesian_trajectory"))
      return;

    if (target_waypoints->points.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No waypoints specified");
      return;
    }

    // Fill in an Eigen::Matrix3xd with the xyz goal
    size_t num_waypoints = target_waypoints->points.size();
    Eigen::Matrix3Xd xyz_positions(3, num_waypoints);
    Eigen::Matrix3Xd euler_angles(3, num_waypoints);
    Eigen::VectorXd times(num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i) {
      times(i) = target_waypoints->points[i].time_from_start.sec + target_waypoints->points[i].time_from_start.nanosec * 1e-9;
      xyz_positions(0, i) = target_waypoints->points[i].positions[0];
      xyz_positions(1, i) = target_waypoints->points[i].positions[1];
      xyz_positions(2, i) = target_waypoints->points[i].positions[2];
      euler_angles(0, i) = target_waypoints->points[i].positions[3];
      euler_angles(1, i) = target_waypoints->points[i].positions[4];
      euler_angles(2, i) = target_waypoints->points[i].positions[5];
    }

    // Replan
    updateSE3Waypoints(use_traj_times_, times, xyz_positions, &euler_angles, true);
  }

  // "Jog" the arm along each joint
  void jointJogCallback(const control_msgs::msg::JointJog::SharedPtr jog_msg) {

    if (!checkArmConditions("joint_jog"))
      return;

    bool inc_vel = true;
    if (jog_msg->velocities.empty()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "No velocities specified... Assuming zero velocity");
      inc_vel = false;
    } 
    else if (jog_msg->velocities.size() != static_cast<size_t>(num_joints_)) {
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

    if (jog_msg->displacements.size() != static_cast<size_t>(num_joints_)) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Displacement size not correct");
      return;
    }

    for (size_t joint = 0; joint < static_cast<size_t>(num_joints_); ++joint) {
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

    if (!checkArmConditions("cartesian_jog"))
      return;

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
    Eigen::Matrix3Xd euler_angles(3, 1);
    Eigen::VectorXd times(1);

    times(0) = jog_msg->duration;
    xyz_positions(0, 0) = cur_pos[0] + jog_msg->displacements[0];
    xyz_positions(1, 0) = cur_pos[1] + jog_msg->displacements[1];
    xyz_positions(2, 0) = cur_pos[2] + jog_msg->displacements[2];
    euler_angles(0, 0) = cur_euler[0];
    euler_angles(1, 0) = cur_euler[1];
    euler_angles(2, 0) = cur_euler[2];

    // Replan
    updateSE3Waypoints(use_traj_times_, times, xyz_positions, &euler_angles, true);
  }

  // "Jog" the target end effector location in SE(3), replanning
  // smoothly to the new location
  // First three entires are linear, and last three are angular
  void SE3JogCallback(const control_msgs::msg::JointJog::SharedPtr jog_msg) {

    if (!checkArmConditions("SE3_jog"))
      return;

    if (jog_msg->displacements.size() != 6) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Displacement size not correct");
      return;
    }
    
    // Get current position and orientation
    // (We use the last position command for smoother motion)
    Eigen::Vector3d cur_pos;
    Eigen::Matrix3d cur_orientation, new_orientation;
    arm_->FK(arm_->lastFeedback().getPositionCommand(), cur_pos, cur_orientation);

    Eigen::Matrix3Xd xyz_positions(3, 1);
    Eigen::Matrix3Xd euler_angles(3, 1);
    Eigen::VectorXd times(1);

    // Calculate the new end-effector orientation
    // World frame's X-axis is End-effector frame's Z-axis: Roll
    // World frame's Y-axis is End-effector frame's X-axis: Pitch
    // World frame's Z-axis is End-effector frame's Y-axis: Yaw
    // In our convention, yaw is global (extrinsic) and roll is local (intrinsic), with respect to the end-effector frame
    times(0) = jog_msg->duration;
    new_orientation = cur_orientation
                      * Eigen::AngleAxisd(jog_msg->displacements[5], Eigen::Vector3d::UnitY()).matrix() 
                      * Eigen::AngleAxisd(jog_msg->displacements[4], Eigen::Vector3d::UnitX()).matrix()
                      * Eigen::AngleAxisd(jog_msg->displacements[3], Eigen::Vector3d::UnitZ()).matrix();

    euler_angles = new_orientation.eulerAngles(0, 1, 2);

    xyz_positions(0, 0) = cur_pos[0] + jog_msg->displacements[0]; // x
    xyz_positions(1, 0) = cur_pos[1] + jog_msg->displacements[1]; // y
    xyz_positions(2, 0) = cur_pos[2] + jog_msg->displacements[2]; // z

    // Replan
    updateSE3Waypoints(use_traj_times_, times, xyz_positions, &euler_angles, false);
  }

  // Control the wrench at the end-effector
  void wrenchCommandCallback(const geometry_msgs::msg::Wrench::SharedPtr wrench_msg)
  {
    if (!checkArmConditions("cmd_ee_wrench"))
      return;
    
    if (num_joints_ != 6)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Wrench control only available for 6-DoF arms");
      return;
    }

    Eigen::VectorXd desired_ee_wrench(6);
    desired_ee_wrench(0) = (*wrench_msg).force.x;
    desired_ee_wrench(1) = (*wrench_msg).force.y;
    desired_ee_wrench(2) = (*wrench_msg).force.z;
    desired_ee_wrench(3) = (*wrench_msg).torque.x;
    desired_ee_wrench(4) = (*wrench_msg).torque.y;
    desired_ee_wrench(5) = (*wrench_msg).torque.z;

    Eigen::MatrixXd ee_jacobian;
    arm_->robotModel().getJacobianEndEffector(arm_->lastFeedback().getPosition(), ee_jacobian);

    // Compute torques from wrench
    cmd_joint_effort_ = ee_jacobian.transpose() * desired_ee_wrench;

    // NOTE: Cannot set pending command here, as there is no assurance it will not get overwritten by arm_->update()
  }

  /////////////////////////// UTILITY FUNCTIONS ///////////////////////////

  void publishState() {
    // Publish Joint State
    const auto& fdbk = arm_->lastFeedback();

    const auto pos = fdbk.getPosition();
    const auto vel = fdbk.getVelocity();
    const auto eff = fdbk.getEffort();

    state_msg_.position.resize(pos.size());
    state_msg_.velocity.resize(vel.size());
    state_msg_.effort.resize(eff.size());
    state_msg_.header.stamp = this->now();
    state_msg_.header.frame_id = "base_link";

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
    pose_msg.header.frame_id = "base_link";
    pose_msg.header.stamp = this->now();
    pose_msg.pose.position.x = cur_pose[0];
    pose_msg.pose.position.y = cur_pose[1];
    pose_msg.pose.position.z = cur_pose[2];
    pose_msg.pose.orientation.x = cur_orientation_quat.x();
    pose_msg.pose.orientation.y = cur_orientation_quat.y();
    pose_msg.pose.orientation.z = cur_orientation_quat.z();
    pose_msg.pose.orientation.w = cur_orientation_quat.w();

    end_effector_pose_publisher_->publish(pose_msg);

    // Publish Raw End-Effector Wrench and Gravity Compensated End-Effector Wrench
    geometry_msgs::msg::WrenchStamped ee_wrench_msg;
    ee_wrench_msg.header.stamp = this->now();
    ee_wrench_msg.header.frame_id = "base_link";
    
    Eigen::MatrixXd ee_jacobian;
    arm_->robotModel().getJacobianEndEffector(pos, ee_jacobian);
    
    if (num_joints_ == 6)
    {
      if (std::fabs(ee_jacobian.determinant()) > 1e-6) {
        Eigen::VectorXd ee_wrench_raw = ee_jacobian.transpose().inverse() * eff;
        
        ee_wrench_msg.wrench.force.x = ee_wrench_raw(0);
        ee_wrench_msg.wrench.force.y = ee_wrench_raw(1);
        ee_wrench_msg.wrench.force.z = ee_wrench_raw(2);
        ee_wrench_msg.wrench.torque.x = ee_wrench_raw(3);
        ee_wrench_msg.wrench.torque.y = ee_wrench_raw(4);
        ee_wrench_msg.wrench.torque.z = ee_wrench_raw(5);
        
        ee_wrench_publisher_->publish(ee_wrench_msg);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Singularity! Cannot compute wrench");
      }
    }

    // Publish Center of Mass
    auto& model = arm_->robotModel();
    Eigen::VectorXd masses;
    robot_model::Matrix4dVector frames;
    model.getMasses(masses);
    model.getFK(robot_model::FrameType::CenterOfMass, pos, frames);

    center_of_mass_message_.m = 0.0;
    Eigen::Vector3d weighted_sum_com = Eigen::Vector3d::Zero();
    for(size_t i = 0; i < model.getFrameCount(robot_model::FrameType::CenterOfMass); ++i) {
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
    for (size_t i = 0; i < command.size(); ++i) {
      command[i].led().set(color);
    }
  }

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const bool use_traj_times, const Eigen::VectorXd& times, const Eigen::MatrixXd& angles, const Eigen::MatrixXd& velocities, const Eigen::MatrixXd& accelerations) {
    // Data sanity check:
    if (angles.rows() != velocities.rows()                    || // Number of joints
        angles.rows() != accelerations.rows()                 ||
        angles.rows() != static_cast<long int>(arm_->size())  ||
        angles.cols() != velocities.cols()                    || // Number of waypoints
        angles.cols() != accelerations.cols()                 ||
        angles.cols() != times.size()                         ||
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
  void updateSE3Waypoints(const bool use_traj_times, const Eigen::VectorXd& times, const Eigen::Matrix3Xd& xyz_positions, const Eigen::Matrix3Xd* euler_angles = nullptr, const bool pureCartesian = false) {
    // Data sanity check:
    if (euler_angles && euler_angles->cols() != xyz_positions.cols())
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

    int min_num_joints = 6;
    if(pureCartesian)
    {
      min_num_joints = 3;
    }

    // For each waypoint, find the joint angles to move to it, starting from the last
    // waypoint, and save into the position vector.
    if (euler_angles && num_joints_ >= min_num_joints) {
      // If we are given tip directions, add these too...
      for (size_t i = 0; i < static_cast<size_t>(num_waypoints); ++i) {

        // Covert euler angles to a 3x3 rotation matrix
        Eigen::Matrix3d rotation_matrix;

        // Following Eigen's convention: 
        // Yaw(Z) (locally) → Pitch(Y) (locally) → Roll(X) (locally)
        // which is equivalent to:
        // Roll(X) (globally) → Pitch(Y) (globally) → Yaw(Z) (globally) 
        // Roll is extrinsic, and yaw is intrinsic
        rotation_matrix = Eigen::AngleAxisd(euler_angles->col(i)[0], Eigen::Vector3d::UnitX()).matrix()
                        * Eigen::AngleAxisd(euler_angles->col(i)[1], Eigen::Vector3d::UnitY()).matrix()
                        * Eigen::AngleAxisd(euler_angles->col(i)[2], Eigen::Vector3d::UnitZ()).matrix();

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
      for (size_t i = 0; i < static_cast<size_t>(num_waypoints); ++i) {
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

  void homeArm() {
    is_homing_ = true;
    // Go to home position
    try {
      arm_->update();
      arm_->setGoal(arm::Goal::createFromPosition(3.0, home_position_));
      RCLCPP_INFO(this->get_logger(), "Homing arm...");
    }
    catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Cannot set goal to home position: %s", e.what());
      is_homing_ = false;
      return;
    }

    // Raise homed flag only after it reaches the home position
    while (!arm_->atGoal() && rclcpp::ok()) {
      update();
    }
    is_homing_ = false;
    RCLCPP_INFO(this->get_logger(), "Reached home position");
  }

  void stopArm() {
    arm_->setGoal(arm::Goal::createFromPosition(arm_->lastFeedback().getPosition())); // Stop the arm at the current position
    RCLCPP_INFO(this->get_logger(), "Arm stopped");
    if (is_homing_) is_homing_ = false; // If we were homing, stop the homing process
  }

  /////////////////// Initialize arm ///////////////////
  bool initializeArm() {

    // Validate the configuration before proceeding
    if (config_package_.empty() || config_file_.empty())
    {
      throw std::runtime_error("Both config_package and config_file must be provided.");
    }

    // Resolve the config file path using the package share directory
    const std::string package_share_directory = ament_index_cpp::get_package_share_directory(config_package_);
    const std::string config_file_path = package_share_directory + "/config/arms/" + config_file_;
    std::vector<std::string> errors;
  
    // Load the config
    const auto arm_config = RobotConfig::loadConfig(config_file_path, errors);
    for (const auto& error : errors) {
      RCLCPP_ERROR(this->get_logger(), error.c_str());
    }
    if (!arm_config) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load configuration from: %s", config_file_path.c_str());
      return false;
    }

    // Create arm from config
    arm_ = arm::Arm::create(*arm_config);
    arm_->update();

    // Terminate if arm not found
    if (!arm_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create arm!");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Arm connected.");
    num_joints_ = arm_->size();

    // Check if home position is provided
    if (arm_config->getUserData().hasFloatList("home_position")) {

      // Check that home_position has the right length
      if (arm_config->getUserData().getFloatList("home_position").size() != num_joints_) {
        RCLCPP_ERROR(this->get_logger(), "HEBI config \"user_data\"'s \"home_position\" field must have the same number of elements as degrees of freedom! Ignoring...");
        home_position_.fill(std::numeric_limits<double>::quiet_NaN());
      }
      else {
        home_position_ = Eigen::Map<Eigen::VectorXd>(arm_config->getUserData().getFloatList("home_position").data(), arm_config->getUserData().getFloatList("home_position").size());
        home_position_available_ = true;
        RCLCPP_INFO(this->get_logger(), "Found and successfully read 'home_position' parameter.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "\"home_position\" not provided in config file. Not traveling to home.");
      home_position_.fill(std::numeric_limits<double>::quiet_NaN());
    }

    // Check if IK seed is provided
    if (arm_config->getUserData().hasFloatList("ik_seed_pos")) {

      // Check if ik_seed has the right length
      if (arm_config->getUserData().getFloatList("ik_seed_pos").size() != num_joints_) {
        RCLCPP_ERROR(this->get_logger(), "HEBI config \"user_data\"'s \"ik_seed\" field must have the same number of elements as degrees of freedom! Ignoring...");
        use_ik_seed_ = false;
      }
      else {
        ik_seed_ = Eigen::Map<Eigen::VectorXd>(arm_config->getUserData().getFloatList("ik_seed_pos").data(), arm_config->getUserData().getFloatList("ik_seed_pos").size());
        RCLCPP_INFO(this->get_logger(), "Found and successfully read 'ik_seed' parameter");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "\"ik_seed\" not provided in config file. Setting use_ik_seed to false.");
      use_ik_seed_ = false;
    }

    // Get the "use_traj_times" for the arm
    if (this->has_parameter("use_traj_times")) {
      this->get_parameter("use_traj_times", use_traj_times_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'use_traj_times' parameter; Setting use_traj_times to true!");
      use_traj_times_ = true;
    }

    // Get the "prefix" for the joint state topic names
    std::string prefix;
    if (this->has_parameter("prefix")) {
      this->get_parameter("prefix", prefix);
    }

    // Make a list of family/actuator formatted names for the JointState publisher
    std::vector<std::string> full_names;
    for (size_t idx = 0; idx < arm_config->getNames().size(); ++idx) {
      full_names.push_back(prefix + arm_config->getNames().at(idx));
    }
    // TODO: Figure out a way to get link names from the arm, so it doesn't need to be input separately
    state_msg_.name = full_names;
    return true;
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