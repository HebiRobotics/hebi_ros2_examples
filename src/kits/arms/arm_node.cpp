#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <hebi_msgs/msg/se3_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
    auto use_gripper_des = rcl_interfaces::msg::ParameterDescriptor{};

    // Parameters passed through config file changed during runtime
    auto ik_seed_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_ik_seed_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_traj_times_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto topic_command_timeout_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto compliant_mode_des = rcl_interfaces::msg::ParameterDescriptor{};

    config_file_des.description = "Config file for the arm of type .cfg.yaml.";
    config_package_des.description = "Package containg the config file.";
    prefix_des.description = "Prefix for the arm.";
    use_gripper_des.description = "Use the gripper if available. If false, the gripper will not be used even if it is available.";
    ik_seed_des.description = "Seed for inverse kinematics. Can be changed during runtime.";
    use_ik_seed_des.description = "Use the stored IK seed position. Can be changed during runtime.";
    use_traj_times_des.description = "Use trajectory times specified in the trajectory messages. If false, a default time is used based on a heuristic. Can be changed during runtime.";
    topic_command_timeout_des.description = "Timeout for topic commands in seconds. If no new command is received within this time, the arm will resume accepting commands from actions or other topics. Can be changed during runtime.";
    compliant_mode_des.description = "No arm motion can be commanded in compliant mode. Can be changed during runtime.";

    // Declare default parameter values
    this->declare_parameter("config_file", "", config_file_des);
    this->declare_parameter("config_package", "", config_package_des);
    this->declare_parameter("prefix", "", prefix_des);
    this->declare_parameter("use_gripper", false, use_gripper_des);
    this->declare_parameter("ik_seed", std::vector<double>(6, std::numeric_limits<double>::quiet_NaN()), ik_seed_des);
    this->declare_parameter("use_ik_seed", false, use_ik_seed_des);
    this->declare_parameter("use_traj_times", true, use_traj_times_des);
    this->declare_parameter("topic_command_timeout", 1.0, topic_command_timeout_des);
    this->declare_parameter("compliant_mode", false, compliant_mode_des);

    // Get the parameters that are passed into the node
    config_package_ = this->get_parameter("config_package").as_string();
    config_file_ = this->get_parameter("config_file").as_string();
    topic_command_timeout_s_ = this->get_parameter("topic_command_timeout").as_double();
    use_gripper_ = this->get_parameter("use_gripper").as_bool();

    // Initialize the arm with configs
    if (!initializeArm()) return;

    // Event handler for parameter changes
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Parameter callbacks
    ik_seed_callback_handle_ = parameter_event_handler_->add_parameter_callback("ik_seed", std::bind(&ArmNode::ikSeedCallback, this, std::placeholders::_1));
    use_ik_seed_callback_handle_ = parameter_event_handler_->add_parameter_callback("use_ik_seed", std::bind(&ArmNode::useIKSeedCallback, this, std::placeholders::_1));
    use_traj_times_callback_handle_ = parameter_event_handler_->add_parameter_callback("use_traj_times", std::bind(&ArmNode::useTrajTimesCallback, this, std::placeholders::_1));
    topic_command_timeout_callback_handle_ = parameter_event_handler_->add_parameter_callback("topic_command_timeout", std::bind(&ArmNode::topicCommandTimeoutCallback, this, std::placeholders::_1));
    compliant_mode_callback_handle_ = parameter_event_handler_->add_parameter_callback("compliant_mode", std::bind(&ArmNode::compliantModeCallback, this, std::placeholders::_1));

    // Subscribers
    joint_jog_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("joint_jog", 10, std::bind(&ArmNode::jointJogCallback, this, std::placeholders::_1));
    cartesian_jog_subscriber_ = this->create_subscription<hebi_msgs::msg::SE3Jog>("cartesian_jog", 10, std::bind(&ArmNode::cartesianJogCallback, this, std::placeholders::_1));
    SE3_jog_subscriber_ = this->create_subscription<hebi_msgs::msg::SE3Jog>("SE3_jog", 10, std::bind(&ArmNode::SE3JogCallback, this, std::placeholders::_1));
    joint_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10, std::bind(&ArmNode::jointWaypointsCallback, this, std::placeholders::_1));
    cartesian_waypoint_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("cartesian_trajectory", 10, std::bind(&ArmNode::cartesianWaypointsCallback, this, std::placeholders::_1));
    cmd_ee_wrench_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>("cmd_ee_wrench", 10, std::bind(&ArmNode::wrenchCommandCallback, this, std::placeholders::_1));
    if (use_gripper_) {
      gripper_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("cmd_gripper", 10, std::bind(&ArmNode::gripperCommandCallback, this, std::placeholders::_1));
    }

    // Publishers
    arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    center_of_mass_publisher_ = this->create_publisher<geometry_msgs::msg::Inertia>("inertia", 10);
    end_effector_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
    ee_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ee_wrench", 10);
    ee_force_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("ee_force", 10);
    goal_progress_publisher_ = this->create_publisher<std_msgs::msg::Float64>("goal_progress", 10);
    if (use_gripper_) {
      gripper_state_publisher_ = this->create_publisher<std_msgs::msg::Float64>("gripper_state", 10);
    }

    // Services
    home_service_ = this->create_service<std_srvs::srv::Trigger>("home", std::bind(&ArmNode::homeCallback, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<std_srvs::srv::Trigger>("stop", std::bind(&ArmNode::stopCallback, this, std::placeholders::_1, std::placeholders::_2));
    if (use_gripper_) {
      gripper_service_ = this->create_service<std_srvs::srv::SetBool>("gripper", std::bind(&ArmNode::gripperCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    
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
      if (gripper_) {
        if (!gripper_->send()) {
          RCLCPP_WARN(this->get_logger(), "Error Sending Gripper Commands -- Check Connection");
        }
      }
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

  Eigen::VectorXd ik_seed_{ Eigen::VectorXd::Constant(6, std::numeric_limits<double>::quiet_NaN()) };
  bool use_ik_seed_{false};

  bool use_traj_times_{true};

  sensor_msgs::msg::JointState state_msg_;
  std_msgs::msg::Float64 gripper_state_msg_;
  geometry_msgs::msg::PoseStamped ee_pose_msg_;
  geometry_msgs::msg::WrenchStamped ee_wrench_msg_;
  geometry_msgs::msg::Vector3Stamped ee_force_msg_;
  geometry_msgs::msg::Inertia center_of_mass_message_;
  std_msgs::msg::Float64 goal_progress_msg_;

  Eigen::Vector3d target_xyz {Eigen::Vector3d::Constant(0.0)};
  Eigen::Matrix3d target_rotmat {Eigen::Matrix3d::Identity()};
  double max_target_radius_{1.0};

  bool has_active_topic_commands_{false};
  std::string active_command_topic_;
  double last_active_command_time_{0.0};
  double topic_command_timeout_s_{1.0};

  bool use_gripper_{false};
  std::unique_ptr<hebi::arm::Gripper> gripper_;

  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<hebi_msgs::msg::SE3Jog>::SharedPtr cartesian_jog_subscriber_;
  rclcpp::Subscription<hebi_msgs::msg::SE3Jog>::SharedPtr SE3_jog_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr cartesian_waypoint_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_waypoint_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr cmd_ee_wrench_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_command_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Inertia>::SharedPtr center_of_mass_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ee_wrench_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr ee_force_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr goal_progress_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_state_publisher_;

  rclcpp_action::Server<hebi_msgs::action::ArmMotion>::SharedPtr action_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_service_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> ik_seed_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> use_ik_seed_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> use_traj_times_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> compliant_mode_callback_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> topic_command_timeout_callback_handle_;

  ////////////////////// PARAMETER CALLBACK FUNCTIONS //////////////////////
  void ikSeedCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(this->get_logger(), "Received an update to parameter 'ik_seed'");

    std::vector<double> ik_seed_vector;
    this->get_parameter("ik_seed", ik_seed_vector);
    if (ik_seed_vector.size() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter is empty; Ignoring!");
    }
    else if (ik_seed_vector.size() != this->arm_->size()) {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter not the same length as HRDF file's number of DoF! Ignoring!");
    }
    else if (std::any_of(ik_seed_vector.begin(), ik_seed_vector.end(), [](double v){ return !std::isfinite(v); })) {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter contains non-finite (NaN or Inf) values! Ignoring!");
    }
    else 
    {
      ik_seed_ = Eigen::VectorXd(arm_->size());
      for (size_t i = 0; i < ik_seed_vector.size(); ++i) {
        ik_seed_[i] = ik_seed_vector[i];
      }
      RCLCPP_INFO(this->get_logger(), "Successfully updated 'ik_seed' parameter");
      if (!use_ik_seed_) {
        RCLCPP_INFO(this->get_logger(), "Enable 'use_ik_seed' parameter to use the updated IK seed");
      }
    }
  }

  void useIKSeedCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type '%s': %s",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_bool() ? "true" : "false");

    bool use_ik_seed;
    this->get_parameter("use_ik_seed", use_ik_seed);
    if (use_ik_seed && ik_seed_.size() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter is empty; Correct it before using 'use_ik_seed'!");
      use_ik_seed_ = false;
    }
    else if (use_ik_seed && ik_seed_.size() != this->arm_->size()) {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter not the same length as HRDF file's number of DoF! Correct it before using 'use_ik_seed'!");
      use_ik_seed_ = false;
    }
    else if (use_ik_seed && std::any_of(ik_seed_.begin(), ik_seed_.end(), [](double v){ return !std::isfinite(v); })) {
      RCLCPP_WARN(this->get_logger(), "'ik_seed' parameter contains non-finite (NaN or Inf) values! Correct it before using 'use_ik_seed'!");
    }
    else {
      use_ik_seed_ = use_ik_seed;
      RCLCPP_INFO(this->get_logger(), "Successfully updated 'use_ik_seed' parameter to %s", use_ik_seed_ ? "true" : "false");
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

  void topicCommandTimeoutCallback(const rclcpp::Parameter & p) {
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type '%s': %f",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());

    this->get_parameter("topic_command_timeout", topic_command_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "Found and successfully updated 'topic_command_timeout' parameter to %f seconds", topic_command_timeout_s_);
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
    if (has_active_topic_commands_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Rejecting arm motion action request - arm is currently receiving topic commands!");
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
    else if (!home_position_available_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Home position was not set in the config file!");
      return;
    }
    else if (compliant_mode_) {
      response->success = false;
      response->message = "Arm in compliant mode, cannot home arm";
    }
    else if (has_active_action_) {
      response->success = false;
      response->message = "Arm is executing an action, cannot home arm";
    }
    else if (is_homing_) {
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
    else if (compliant_mode_) {
      response->success = false;
      response->message = "Arm in compliant mode, nothing to stop";
    }
    else if (has_active_action_) {
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

  // Service callback for gripper control
  void gripperCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    (void)request;

    response->success = true;

    if (!gripper_) {
      response->success = false;
      response->message = "Gripper not available, cannot control gripper";
    }

    if (!response->success) {
      RCLCPP_ERROR_STREAM(this->get_logger(), response->message);
      return;
    }

    if (!request->data) {
      gripper_->open();
      response->message = "Gripper opened";
    } else {
      gripper_->close();
      response->message = "Gripper closed";
    }
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

    if (has_active_topic_commands_ && active_command_topic_ != topic_name) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Arm is currently receiving topic commands at " << active_command_topic_ << ", ignoring " << topic_name);
      return false;
    }
    
    if (!has_active_topic_commands_) {
      // Set the target position to current position before starting a new command
      arm_->FK(arm_->lastFeedback().getPositionCommand(), target_xyz, target_rotmat);
    }

    // Set the active command topic and time
    active_command_topic_ = topic_name;
    last_active_command_time_ = this->now().seconds();
    has_active_topic_commands_ = true;

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
  void cartesianJogCallback(const hebi_msgs::msg::SE3Jog::SharedPtr jog_msg) {

    if (!checkArmConditions("cartesian_jog"))
      return;

    Eigen::VectorXd times(1);
    times(0) = jog_msg->duration;

    // Get current position and orientation
    // (We use the last position command for smoother motion)
    Eigen::Vector3d cur_pos;  // Not used
    Eigen::Matrix3d cur_orientation;
    arm_->FK(arm_->lastFeedback().getPositionCommand(), cur_pos, cur_orientation);

    // Convert orientation to Euler angles
    Eigen::Matrix3Xd cur_euler = cur_orientation.eulerAngles(0, 1, 2);

    target_xyz[0] += jog_msg->dx;
    target_xyz[1] += jog_msg->dy;
    target_xyz[2] += jog_msg->dz;
    // Limit the target position to be within the maximum radius
    target_xyz *= std::min(1.0, max_target_radius_ / target_xyz.norm());

    // Replan
    updateSE3Waypoints(use_traj_times_, times, target_xyz, &cur_euler, true);
  }

  // "Jog" the target end effector location in SE(3), replanning
  // smoothly to the new location
  // First three entires are linear, and last three are angular
  void SE3JogCallback(const hebi_msgs::msg::SE3Jog::SharedPtr jog_msg) {

    if (!checkArmConditions("SE3_jog"))
      return;

    Eigen::VectorXd times(1);
    times(0) = jog_msg->duration;

    // Calculate the new end-effector orientation
    // World frame's X-axis is End-effector frame's Z-axis: Roll
    // World frame's Y-axis is End-effector frame's X-axis: Pitch
    // World frame's Z-axis is End-effector frame's Y-axis: Yaw
    // In our convention, yaw is global (extrinsic) and roll is local (intrinsic), with respect to the end-effector frame
    target_rotmat = target_rotmat
                    * Eigen::AngleAxisd(jog_msg->dyaw, Eigen::Vector3d::UnitY()).matrix()   // Yaw
                    * Eigen::AngleAxisd(jog_msg->dpitch, Eigen::Vector3d::UnitX()).matrix() // Pitch
                    * Eigen::AngleAxisd(jog_msg->droll, Eigen::Vector3d::UnitZ()).matrix(); // Roll

    // Convert orientation to Euler angles
    Eigen::Matrix3Xd euler_angles = target_rotmat.eulerAngles(0, 1, 2);

    target_xyz[0] += jog_msg->dx;
    target_xyz[1] += jog_msg->dy;
    target_xyz[2] += jog_msg->dz;
    // Limit the target position to be within the maximum radius
    target_xyz *= std::min(1.0, max_target_radius_ / target_xyz.norm());

    // Replan
    updateSE3Waypoints(use_traj_times_, times, target_xyz, &euler_angles, false);
  }

  // Control the wrench at the end-effector
  void wrenchCommandCallback(const geometry_msgs::msg::Wrench::SharedPtr wrench_msg)
  {
    if (!checkArmConditions("cmd_ee_wrench"))
      return;

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

  // Control the gripper
  void gripperCommandCallback(const std_msgs::msg::Float64::SharedPtr gripper_msg) {
    if (!gripper_) {
      RCLCPP_ERROR(this->get_logger(), "Gripper not enabled in this arm configuration! Ignoring command");
      return;
    }

    // Set the gripper position
    gripper_->setState(gripper_msg->data);
  }

  /////////////////////////// UTILITY FUNCTIONS ///////////////////////////

  void publishState() {
    // Publish Joint State
    const auto& fdbk = arm_->lastFeedback();

    const auto pos = fdbk.getPosition();
    const auto vel = fdbk.getVelocity();
    const auto eff = fdbk.getEffort();

    const int gripper_size = use_gripper_ ? 1 : 0;

    state_msg_.position.resize(pos.size() + gripper_size);
    state_msg_.velocity.resize(vel.size() + gripper_size);
    state_msg_.effort.resize(eff.size() + gripper_size);
    state_msg_.header.stamp = this->now();
    state_msg_.header.frame_id = "base_link";

    Eigen::VectorXd::Map(&state_msg_.position[0], pos.size()) = pos;
    Eigen::VectorXd::Map(&state_msg_.velocity[0], vel.size()) = vel;
    Eigen::VectorXd::Map(&state_msg_.effort[0], eff.size()) = eff;

    if (use_gripper_) {
      state_msg_.position.back() = gripper_->getState();
      state_msg_.velocity.back() = std::numeric_limits<double>::quiet_NaN(); // Gripper velocity not available
      state_msg_.effort.back() = std::numeric_limits<double>::quiet_NaN(); // Gripper effort not available
    }

    arm_state_pub_->publish(state_msg_);

    // Publish Gripper State
    if (use_gripper_) {
      gripper_state_msg_.data = gripper_->getState();
      gripper_state_publisher_->publish(gripper_state_msg_);
    }

    // Publish End Effector Pose
    Eigen::Vector3d cur_pose;
    Eigen::Matrix3d cur_orientation;
    arm_->FK(pos, cur_pose, cur_orientation);
    Eigen::Quaterniond cur_orientation_quat(cur_orientation);

    ee_pose_msg_.header.frame_id = "base_link";
    ee_pose_msg_.header.stamp = this->now();
    ee_pose_msg_.pose.position.x = cur_pose[0];
    ee_pose_msg_.pose.position.y = cur_pose[1];
    ee_pose_msg_.pose.position.z = cur_pose[2];
    ee_pose_msg_.pose.orientation.x = cur_orientation_quat.x();
    ee_pose_msg_.pose.orientation.y = cur_orientation_quat.y();
    ee_pose_msg_.pose.orientation.z = cur_orientation_quat.z();
    ee_pose_msg_.pose.orientation.w = cur_orientation_quat.w();

    end_effector_pose_publisher_->publish(ee_pose_msg_);

    // Publish Raw End-Effector Wrench    
    Eigen::MatrixXd ee_jacobian;
    arm_->robotModel().getJacobianEndEffector(pos, ee_jacobian);

    // Print a warning if the Jacobian is singular
    if (abs(ee_jacobian.transpose().determinant()) < 1e-6) {
      RCLCPP_WARN(this->get_logger(), "Jacobian is singular, end-effector wrench may not be accurate");
    }

    // Calculate pseudo-inverse of J^T
    Eigen::MatrixXd ee_jacobian_t_pinv = ee_jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse();
    // Calculate wrench as (J^T)^-1 * joint effort errors
    Eigen::VectorXd eff_cmd = fdbk.getEffortCommand();
    Eigen::VectorXd ee_wrench = ee_jacobian_t_pinv * (eff_cmd - eff);
    
    ee_wrench_msg_.header.stamp = this->now();
    ee_wrench_msg_.header.frame_id = "base_link";
    ee_wrench_msg_.wrench.force.x = ee_wrench(0);
    ee_wrench_msg_.wrench.force.y = ee_wrench(1);
    ee_wrench_msg_.wrench.force.z = ee_wrench(2);
    ee_wrench_msg_.wrench.torque.x = ee_wrench(3);
    ee_wrench_msg_.wrench.torque.y = ee_wrench(4);
    ee_wrench_msg_.wrench.torque.z = ee_wrench(5);
    
    ee_wrench_publisher_->publish(ee_wrench_msg_);

    // Calculate end-effector force based on end-effector position error
    Eigen::Vector3d ee_pos_cmd;
    Eigen::Matrix3d ee_orientation_cmd;
    arm_->FK(fdbk.getPositionCommand(), ee_pos_cmd, ee_orientation_cmd);
    Eigen::Vector3d pos_error = cur_pose - ee_pos_cmd;

    ee_force_msg_.header.stamp = this->now();
    ee_force_msg_.header.frame_id = "base_link";
    ee_force_msg_.vector.x = pos_error(0);
    ee_force_msg_.vector.y = pos_error(1);
    ee_force_msg_.vector.z = pos_error(2);

    ee_force_publisher_->publish(ee_force_msg_);

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

    // Publish Goal Progress
    goal_progress_msg_.data = arm_->goalProgress();
    goal_progress_publisher_->publish(goal_progress_msg_);

    // Reset active command state
    if (this->now().seconds() - last_active_command_time_ > topic_command_timeout_s_) {
      has_active_topic_commands_ = false;
      active_command_topic_ = "";
      last_active_command_time_ = 0.0;
    }
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
    // Check if last_position contains NaN values
    // If it does, we will use position feedback instead
    else if (last_position.hasNaN()) {
      RCLCPP_DEBUG(this->get_logger(), "Last position command contains NaN values, using last feedback position instead");
      last_position = arm_->lastFeedback().getPosition();
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

  /////////////////// Initialize arm and gripper ///////////////////
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
    // Terminate if arm not found
    if (!arm_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create arm! Please check if the modules are available on the network and the config file is correct.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Arm connected.");
    num_joints_ = arm_->size();
    arm_->update();

    auto arm_config_user_data = arm_config->getUserData();

    // Check if home position is provided
    if (arm_config_user_data.hasFloatList("home_position")) {

      // Check that home_position has the right length
      if (arm_config_user_data.getFloatList("home_position").size() != num_joints_) {
        RCLCPP_ERROR(this->get_logger(), "HEBI config \"user_data\"'s \"home_position\" field must have the same number of elements as degrees of freedom! Ignoring...");
        home_position_.fill(std::numeric_limits<double>::quiet_NaN());
      }
      else {
        home_position_ = Eigen::Map<Eigen::VectorXd>(arm_config_user_data.getFloatList("home_position").data(), arm_config_user_data.getFloatList("home_position").size());
        home_position_available_ = true;
        RCLCPP_INFO(this->get_logger(), "Found and successfully read 'home_position' parameter.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "\"home_position\" not provided in config file. Not traveling to home.");
      home_position_.fill(std::numeric_limits<double>::quiet_NaN());
    }

    // Get the "use_ik_seed" parameter
    if (this->has_parameter("use_ik_seed")) {
      this->get_parameter("use_ik_seed", use_ik_seed_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not find/read 'use_ik_seed' parameter; Setting use_ik_seed to false!");
      use_ik_seed_ = false;
    }
    
    // Check if IK seed is provided
    if (arm_config_user_data.hasFloatList("ik_seed_pos")) {
      // Check if ik_seed has the right length
      auto ik_seed_pos = arm_config_user_data.getFloatList("ik_seed_pos");
      if (ik_seed_pos.empty()) {
        RCLCPP_ERROR(this->get_logger(), "HEBI config \"user_data\"'s \"ik_seed\" field is empty! Ignoring...");
        use_ik_seed_ = false;
      }
      else if (ik_seed_pos.size() != num_joints_) {
        RCLCPP_ERROR(this->get_logger(), "HEBI config \"user_data\"'s \"ik_seed\" field must have the same number of elements as degrees of freedom! Ignoring...");
        use_ik_seed_ = false;
      }
      else if (std::any_of(ik_seed_pos.begin(), ik_seed_pos.end(), [](float val) { return !std::isfinite(val); })) {
        RCLCPP_ERROR(this->get_logger(), "HEBI config \"user_data\"'s \"ik_seed\" field contains non-finite values! Ignoring...");
        use_ik_seed_ = false;
      }
      else {
        ik_seed_ = Eigen::Map<Eigen::VectorXd>(ik_seed_pos.data(), ik_seed_pos.size());
        RCLCPP_INFO(this->get_logger(), "Found and successfully read 'ik_seed' parameter. Set `use_ik_seed` parameter to true to use it.");
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "\"ik_seed_pos\" not provided in config file. Not using IK seed.");
      use_ik_seed_ = false; // Default to false if not provided
      ik_seed_.fill(std::numeric_limits<double>::quiet_NaN());
    }

    this->set_parameter(rclcpp::Parameter("use_ik_seed", use_ik_seed_));
    this->set_parameter(rclcpp::Parameter("ik_seed", std::vector<double>(ik_seed_.data(), ik_seed_.data() + ik_seed_.size())));

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

    // Create gripper if requested
    std::string gripper_name = "gripperSpool";  // Kept outside the if statement to use it later in state_msg names
    if (use_gripper_) {
      // Get the "has_gripper" parameter
      if (!arm_config_user_data.hasBool("has_gripper")) {
        RCLCPP_WARN(this->get_logger(), "Gripper not configured in the arm config file, ignoring gripper commands");
        use_gripper_ = false;
        return true; // No gripper to initialize
      }
      if (!arm_config_user_data.getBool("has_gripper")) {
        RCLCPP_WARN(this->get_logger(), "`has_gripper` is set to false in the arm config file, ignoring gripper commands");
        use_gripper_ = false;
        return true; // No gripper to initialize
      }

      std::string gripper_family = arm_config->getFamilies()[0];
      if (arm_config_user_data.hasString("gripper_family")) {
        gripper_family = arm_config_user_data.getString("gripper_family");
      }
      if (arm_config_user_data.hasString("gripper_name")) {
        gripper_name = arm_config_user_data.getString("gripper_name");
      }
      double gripper_close_effort = 1.0;
      if (arm_config_user_data.hasFloat("gripper_close_effort")) {
        gripper_close_effort = arm_config_user_data.getFloat("gripper_close_effort");
      }
      double gripper_open_effort = -5.0;
      if (arm_config_user_data.hasFloat("gripper_open_effort")) {
        gripper_open_effort = arm_config_user_data.getFloat("gripper_open_effort");
      }

      // Create gripper from config
      gripper_ = arm::Gripper::create(
        gripper_family,
        gripper_name,
        gripper_close_effort,
        gripper_open_effort
      );

      if (!gripper_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create gripper! Please check if the gripper is available on the network and the config file is correct.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Gripper connected.");

      std::string gripper_gains_file = arm_config->getGains("gripper");
      if (!gripper_gains_file.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loading gripper gains from: %s", gripper_gains_file.c_str());
        if (gripper_->loadGains(gripper_gains_file)) {
          RCLCPP_INFO(this->get_logger(), "Gripper gains loaded successfully.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to load gripper gains");
          return false;
        }
      }

      gripper_->open(); // Open the gripper by default
      if (!gripper_->send()) {
        RCLCPP_WARN(this->get_logger(), "Could not send gripper command! Please check connection");
      }
    }

    // Make a list of family/actuator formatted names for the JointState publisher
    std::vector<std::string> full_names;
    for (size_t idx = 0; idx < arm_config->getNames().size(); ++idx) {
      full_names.push_back(prefix + arm_config->getNames().at(idx));
    }
    if (use_gripper_) {
      full_names.push_back(prefix + gripper_name); // Add gripper name to the list
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