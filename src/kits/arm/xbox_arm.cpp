#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/msg/joint_jog.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <hebi_msgs/action/arm_motion.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif


namespace hebi {
namespace ros {

class XBoxArm : public rclcpp::Node {
public:
  
  XBoxArm() : Node("xbox_arm") {

        // TODO : Parameter retrieval
        // Read parameters from robot's yaml
        // this->declare_parameter("names", rclcpp::PARAMETER_STRING_ARRAY);
        // this->declare_parameter("families", rclcpp::PARAMETER_STRING_ARRAY);
        // this->declare_parameter("gains_package", rclcpp::PARAMETER_STRING);
        // this->declare_parameter("gains_file", rclcpp::PARAMETER_STRING);
        // this->declare_parameter("hrdf_package", rclcpp::PARAMETER_STRING);
        // this->declare_parameter("hrdf_file", rclcpp::PARAMETER_STRING);
        // this->declare_parameter("home_position", rclcpp::PARAMETER_DOUBLE_ARRAY);
        // this->declare_parameter("ik_seed", rclcpp::PARAMETER_DOUBLE_ARRAY);
        // this->declare_parameter("prefix", "");
        // this->declare_parameter("use_traj_times", true);

        // get_parameter("names", joint_names_);
        // // get_parameter("home_position", home_position_);

        // home_position_ = get_parameter("home_position").get_parameter_value().get<std::vector<double>>();

        // if (this->has_parameter("home_position")) {
        // this->get_parameter("home_position", home_position_);
        // RCLCPP_INFO(this->get_logger(), "Found and successfully read 'home_position' parameter");
        // } else {
        // RCLCPP_WARN(this->get_logger(), "Could not find/read 'home_position' parameter; defaulting to all zeros!");
        // }

        // RCLCPP_INFO(this->get_logger(), "%s, %ld", joint_names_.at(0), joint_names_.size());
        // RCLCPP_INFO(this->get_logger(), "%f, %ld", home_position_.at(0), home_position_.size());
        // RCLCPP_INFO(this->get_logger(), "%ld", home_position_.size());

        joint_names_.push_back("J1_base");
        joint_names_.push_back("J2_shoulder");
        joint_names_.push_back("J3_elbow");
        joint_names_.push_back("J4_wrist1");
        joint_names_.push_back("J5_wrist2");
        joint_names_.push_back("J6_wrist3");

        // Initialize axes vector
        std::vector<float> axes_0 = {-0.0f, -0.0f, 1.0f, -0.0f, -0.0f, 1.0f, 0.0f, 0.0f};
        controller_state_.axes = axes_0;

        // Initialize buttons vector
        std::vector<int> buttons_0 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        controller_state_.buttons = buttons_0;
        
        // Timer
        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0/rate)),
        std::bind(&XBoxArm::timer_callback, this));

        // Publishers
        joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("joint_jog", 50);
        cartesian_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("cartesian_jog", 50);
        se3_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("se3_jog", 50);

        // Subscribers
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&XBoxArm::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Initialized");
    }

private:
  
    // Create Objects
    double rate = 200.0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr cartesian_jog_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr se3_jog_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    // Initialize controller state
    sensor_msgs::msg::Joy controller_state_;

    std::vector<std::string> joint_names_;
    std::vector<double> home_position_;

    double x_vel_ = 0.0;
    double y_vel_ = 0.0;
    double z_vel_ = 0.0;
    double roll_vel_ = 0.0;
    double pitch_vel_ = 0.0;
    double yaw_vel_ = 0.0;

    double x_vel_gain_ = 0.002;
    double y_vel_gain_ = 0.002;
    double z_vel_gain_ = 0.002;
    double roll_vel_gain_ = 0.02;
    double pitch_vel_gain_ = 0.015;
    double yaw_vel_gain_ = 0.015;

    double pitch_max_ = M_PI / 2.0;
    double pitch_pos_ = 0.0;

    double low_pass_admittance_ = 0.05;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Update the controller state
        controller_state_ = *msg;
    }

    void process_controller()
    {
        // Fowards (+X) and backwards (-X) motion through Left Joystick
        x_vel_ += low_pass_admittance_ * (x_vel_gain_ * controller_state_.axes.at(1) - x_vel_);

        // Left (+Y) and right (-Y) motion through Left Joystick
        y_vel_ += low_pass_admittance_ * (y_vel_gain_ * controller_state_.axes.at(0) - y_vel_);

        // Up (+Z) and down (-Z) motion through L1 and L2
        // Remain stationary if neither pressed
        if (controller_state_.buttons.at(4) == 0 && 
            static_cast<int>(controller_state_.axes.at(2)) == 1)
        {
            z_vel_ += low_pass_admittance_ * (0 - z_vel_);
        }
        // Move up if L1 pressed
        else if (controller_state_.buttons.at(4) == 1)
        {
            z_vel_ += low_pass_admittance_ * (z_vel_gain_ - z_vel_);
        }
        // Move down if L2 pressed
        else if (static_cast<int>(controller_state_.axes.at(2)) == -1)
        {
            z_vel_ += low_pass_admittance_ * (-z_vel_gain_ - z_vel_);
        }

        // Roll clockwise (+X) and counterclockwise (-X) through arrow keys
        // Return to zero if neither pressed
        if (static_cast<int>(controller_state_.axes.at(6)) == 0)
        {
            roll_vel_ = 0; //TOFIX
        }
        // Roll clockwise if right key pressed
        else if (static_cast<int>(controller_state_.axes.at(6)) == -1)
        {
            roll_vel_ = roll_vel_gain_;
        }
        // Roll counterclockwise if left key pressed
        else if (static_cast<int>(controller_state_.axes.at(6)) == 1)
        {
            roll_vel_ = -roll_vel_gain_;
        }

        // Pitch up (+Y) and down (-Y) through Right Joystick
        pitch_vel_ = -pitch_vel_gain_ * controller_state_.axes.at(4);

        yaw_vel_ = -yaw_vel_gain_ * -controller_state_.axes.at(3);
    }

    void joint_jog_pub()
    {
        // Initialize Joint Jog Message
        control_msgs::msg::JointJog joint_jog_msg;

        joint_jog_msg.header.stamp = get_clock()->now();
        joint_jog_msg.header.frame_id = "";
        joint_jog_msg.duration = 1.0/rate; // seconds

        joint_jog_msg.joint_names = joint_names_;

        // Angular displacements
        for (size_t joint_idx = 0; joint_idx < joint_jog_msg.joint_names.size(); joint_idx++)
        {
            joint_jog_msg.displacements.push_back(0.0);
        }
        joint_jog_msg.displacements.at(3) = pitch_vel_;
        joint_jog_msg.displacements.at(4) = yaw_vel_;

        // Velocities should be the same dimension as number of joints
        for (size_t joint_idx = 0; joint_idx < joint_jog_msg.joint_names.size(); joint_idx++)
        {
            joint_jog_msg.velocities.push_back(0.0);
        }

        joint_jog_publisher_->publish(joint_jog_msg);
    }

    void cartesian_jog_pub()
    {
        // Initialize Cartesian Jog Message
        control_msgs::msg::JointJog cartesian_jog_msg;

        cartesian_jog_msg.header.stamp = get_clock()->now();
        cartesian_jog_msg.header.frame_id = "";
        cartesian_jog_msg.duration = 1.0/rate; // seconds

        cartesian_jog_msg.joint_names = {"x", "y", "z"};

        // Linear displacements
        for (size_t dim_idx = 0; dim_idx < cartesian_jog_msg.joint_names.size(); dim_idx++)
        {
            cartesian_jog_msg.displacements.push_back(0.0);
        }
        cartesian_jog_msg.displacements.at(0) = x_vel_;
        cartesian_jog_msg.displacements.at(1) = y_vel_;
        cartesian_jog_msg.displacements.at(2) = z_vel_;

        // Velocities should be the same dimension as number of directions
        for (size_t dim_idx = 0; dim_idx < cartesian_jog_msg.joint_names.size(); dim_idx++)
        {
            cartesian_jog_msg.velocities.push_back(0.0);
        }
        cartesian_jog_msg.velocities.at(0) = x_vel_;
        cartesian_jog_msg.velocities.at(1) = y_vel_;
        cartesian_jog_msg.velocities.at(2) = z_vel_;

        cartesian_jog_publisher_->publish(cartesian_jog_msg);
    }

    void se3_jog_pub()
    {
        // Initialize SE(3) Jog Message
        control_msgs::msg::JointJog se3_jog_msg;

        se3_jog_msg.header.stamp = get_clock()->now();
        se3_jog_msg.header.frame_id = "";
        se3_jog_msg.duration = 1.0/rate; // seconds

        se3_jog_msg.joint_names = {"roll", "pitch", "yaw", "x", "y", "z"};

        // Displacements
        for (size_t dof_idx = 0; dof_idx < se3_jog_msg.joint_names.size(); dof_idx++)
        {
            se3_jog_msg.displacements.push_back(0.0);
        }
        se3_jog_msg.displacements.at(0) = roll_vel_;
        se3_jog_msg.displacements.at(1) = pitch_vel_;
        se3_jog_msg.displacements.at(2) = yaw_vel_;
        se3_jog_msg.displacements.at(3) = x_vel_;
        se3_jog_msg.displacements.at(4) = y_vel_;
        se3_jog_msg.displacements.at(5) = z_vel_;

        // Velocities should be the same dimension as number of dof
        for (size_t dof_idx = 0; dof_idx < se3_jog_msg.joint_names.size(); dof_idx++)
        {
            se3_jog_msg.velocities.push_back(0.0);
        }
        se3_jog_msg.velocities.at(0) = roll_vel_;
        se3_jog_msg.velocities.at(1) = pitch_vel_;
        se3_jog_msg.velocities.at(2) = yaw_vel_;
        se3_jog_msg.velocities.at(3) = x_vel_;
        se3_jog_msg.velocities.at(4) = y_vel_;
        se3_jog_msg.velocities.at(5) = z_vel_;

        se3_jog_publisher_->publish(se3_jog_msg);
    }

    void timer_callback()
    {   
        // Decide Jog values based on controller state
        process_controller();
            
        // Publish Jog Messages accordingly
        // cartesian_jog_pub();
        se3_jog_pub();
    }
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::XBoxArm>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}