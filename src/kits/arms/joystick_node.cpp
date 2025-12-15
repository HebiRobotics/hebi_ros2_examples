#include "rclcpp/rclcpp.hpp"
#include <hebi_msgs/msg/se3_jog.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hebi
{
namespace ros
{

class JoystickNode : public rclcpp::Node
{
public:
  JoystickNode() : Node("joystick_node"), num_joints_initialized_(false)
  {

    this->declare_parameter("prefix", "");
    this->get_parameter("prefix", prefix_);

    // Initialize axes vector
    std::vector<float> axes_0 = {-0.0f, -0.0f, 1.0f, -0.0f, -0.0f, 1.0f, 0.0f, 0.0f};
    controller_state_.axes = axes_0;

    // Initialize buttons vector
    std::vector<int> buttons_0 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    controller_state_.buttons = buttons_0;

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
        std::bind(&JoystickNode::timer_callback, this));

    // Publishers
    cartesian_jog_publisher_ = this->create_publisher<hebi_msgs::msg::SE3Jog>(prefix_ + "/cartesian_jog", 50);
    SE3_jog_publisher_ = this->create_publisher<hebi_msgs::msg::SE3Jog>(prefix_ + "/SE3_jog", 50);

    // Subscribers
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickNode::joy_callback, this, std::placeholders::_1));

    // Joint state subscriber used to determine number of joints
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(prefix_ + "/joint_states", 10, std::bind(&JoystickNode::get_num_joints, this, std::placeholders::_1));

    // Home Service Client
    home_client_ = this->create_client<std_srvs::srv::Trigger>("home");

    // Initialize last button press time
    last_button_press_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Started Joystick Arm Controller");
    RCLCPP_INFO(this->get_logger(), "Controller mapping:");
    RCLCPP_INFO(this->get_logger(), "  Left Stick: X/Y translation");
    RCLCPP_INFO(this->get_logger(), "  Right Stick: Pitch/Yaw rotation");
    RCLCPP_INFO(this->get_logger(), "  R1/RB: Z up");
    RCLCPP_INFO(this->get_logger(), "  R2/RT: Z down");
    RCLCPP_INFO(this->get_logger(), "  D-pad L/R: Roll");
    RCLCPP_INFO(this->get_logger(), "  Start: Go to home position");
  }

private:
  // Create Objects
  double rate = 200.0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hebi_msgs::msg::SE3Jog>::SharedPtr cartesian_jog_publisher_;
  rclcpp::Publisher<hebi_msgs::msg::SE3Jog>::SharedPtr SE3_jog_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_client_;

  // Initialize controller state
  sensor_msgs::msg::Joy controller_state_;

  int num_joints_;
  bool num_joints_initialized_;
  std::string prefix_;

  // Last button press time for preventing multiple home requests
  rclcpp::Time last_button_press_time_;

  // Initialize SE(3) velocities
  double x_vel_ = 0.0;
  double y_vel_ = 0.0;
  double z_vel_ = 0.0;
  double roll_vel_ = 0.0;
  double pitch_vel_ = 0.0;
  double yaw_vel_ = 0.0;

  // Initialize tuned gain values for velocities
  double x_vel_gain_ = 0.002;
  double y_vel_gain_ = 0.002;
  double z_vel_gain_ = 0.002;
  double roll_vel_gain_ = 0.03;
  double pitch_vel_gain_ = 0.015;
  double yaw_vel_gain_ = 0.015;

  // Initialize tuned admittance value for low pass filtering
  double low_pass_admittance_ = 0.05;

  // Deadband threshold to prevent joystick drift
  double deadband_threshold_ = 0.1;

  void get_num_joints(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!num_joints_initialized_)
    {
      num_joints_ = msg->name.size();
      num_joints_initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "Detected %d joints", num_joints_);

      joint_state_subscriber_.reset(); // Only need to subscribe once
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Update the controller state
    controller_state_ = *msg;
  }

  // Apply deadband to joystick axes to prevent drift
  double apply_deadband(double value)
  {
    return (std::abs(value) < deadband_threshold_) ? 0.0 : value;
  }

  void process_controller()
  {
    // === TRANSLATION CONTROL (Left Stick) ===
    // Forward (+X) and backward (-X) motion through Left Joystick Y-axis
    double x_input = apply_deadband(controller_state_.axes.at(1));
    x_vel_ += low_pass_admittance_ * (x_vel_gain_ * x_input - x_vel_);

    // Left (+Y) and right (-Y) motion through Left Joystick X-axis
    double y_input = apply_deadband(controller_state_.axes.at(0));
    y_vel_ += low_pass_admittance_ * (y_vel_gain_ * y_input - y_vel_);

    // === Z-AXIS CONTROL (Right Shoulder/Trigger) ===
    // Up (+Z) with R1/RB button (index 5), Down (-Z) with R2/RT trigger (axis 5)
    double z_target = 0.0;

    // R1/RB button for up
    if (controller_state_.buttons.at(5) == 1)
    {
      z_target = z_vel_gain_;
    }
    // R2/RT trigger for down (axis 5: 1.0 = not pressed, -1.0 = fully pressed)
    else if (controller_state_.axes.at(5) < 0.5)
    {
      // Map trigger from [1.0, -1.0] to [0.0, -z_vel_gain_]
      z_target = -z_vel_gain_ * (1.0 - controller_state_.axes.at(5)) / 2.0;
    }

    z_vel_ += low_pass_admittance_ * (z_target - z_vel_);

    // === ROTATION CONTROL ===
    // Roll clockwise (+) and counterclockwise (-) through D-pad left/right (axis 6)
    double roll_target = 0.0;
    int dpad_lr = static_cast<int>(controller_state_.axes.at(6));
    if (dpad_lr == 1)  // D-pad left
    {
      roll_target = -roll_vel_gain_;
    }
    else if (dpad_lr == -1)  // D-pad right
    {
      roll_target = roll_vel_gain_;
    }
    roll_vel_ += low_pass_admittance_ * (roll_target - roll_vel_);

    // Pitch up (+) and down (-) through Right Joystick Y-axis
    // Inverted for natural control (push forward = pitch down)
    double pitch_input = apply_deadband(controller_state_.axes.at(4));
    pitch_vel_ += low_pass_admittance_ * (-pitch_vel_gain_ * pitch_input - pitch_vel_);

    // Yaw left (+) and right (-) through Right Joystick X-axis
    double yaw_input = apply_deadband(controller_state_.axes.at(3));
    yaw_vel_ += low_pass_admittance_ * (yaw_vel_gain_ * yaw_input - yaw_vel_);
  }

  void cartesian_jog_pub()
  {
    // Initialize Cartesian Jog Message
    hebi_msgs::msg::SE3Jog cartesian_jog_msg;

    cartesian_jog_msg.header.stamp = get_clock()->now();
    cartesian_jog_msg.header.frame_id = "";
    cartesian_jog_msg.duration = 1.0 / rate; // seconds

    cartesian_jog_msg.dx = x_vel_;
    cartesian_jog_msg.dy = y_vel_;
    cartesian_jog_msg.dz = z_vel_;

    cartesian_jog_publisher_->publish(cartesian_jog_msg);
  }

  void SE3_jog_pub()
  {
    // Initialize SE(3) Jog Message
    hebi_msgs::msg::SE3Jog SE3_jog_msg;

    SE3_jog_msg.header.stamp = get_clock()->now();
    SE3_jog_msg.header.frame_id = "";
    SE3_jog_msg.duration = 5.0 / rate; // seconds

    SE3_jog_msg.dx = x_vel_;
    SE3_jog_msg.dy = y_vel_;
    SE3_jog_msg.dz = z_vel_;
    SE3_jog_msg.droll = roll_vel_;
    SE3_jog_msg.dpitch = pitch_vel_;
    SE3_jog_msg.dyaw = yaw_vel_;

    SE3_jog_publisher_->publish(SE3_jog_msg);
  }

  void go_home()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // Wait for the service to be available
    int try_count = 0;
    while (!home_client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      try_count++;
      if (try_count > 3)
      {
        RCLCPP_ERROR(this->get_logger(), "Service not available after 3 tries. Exiting.");
        return;
      }
    }
    auto result = home_client_->async_send_request(request);
  }

  void timer_callback()
  {
    if (!num_joints_initialized_)
    {
      return;
    }
    
    // Decide Jog values based on controller state
    process_controller();

    // Re-home if START Button is pressed
    if (controller_state_.buttons.at(7) == 1 &&
        (this->now() - last_button_press_time_).seconds() > 1.0)
    {
      go_home();
      last_button_press_time_ = this->now();
    }

    // Publish Jog Messages accordingly
    // Publish SE(3) jog messages only when there are enough degrees of freedom
    if (num_joints_ >= 6)
    {
      SE3_jog_pub();
    }
    else
    {
      cartesian_jog_pub();
    }
  }
};

} // namespace ros
} // namespace hebi

int main(int argc, char **argv)
{

  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::JoystickNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}