#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/msg/joint_jog.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <hebi_msgs/action/arm_motion.hpp>
#include <Eigen/Dense>


#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif


namespace hebi {
namespace ros {

class MagnetArm : public rclcpp::Node {
public:
  
  MagnetArm() : Node("magnet_arm") {

        // Read parameters from robot's yaml
        this->declare_parameter("names", rclcpp::PARAMETER_STRING_ARRAY);
        this->declare_parameter("families", rclcpp::PARAMETER_STRING_ARRAY);
        this->declare_parameter("home_position", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("prefix", "");

        if (this->has_parameter("home_position")) {
            this->get_parameter("home_position", home_position_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not find/read 'home_position' parameter; defaulting to all zeros!");
        }

        if (this->has_parameter("names")) {
            this->get_parameter("names", joint_names_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not find/read 'names' parameter; defaulting to all zeros!");
        }

        // TF Listener and Buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0/rate)),
        std::bind(&MagnetArm::timer_callback, this));

        // Publishers
        joint_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("joint_jog", 50);
        cartesian_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("cartesian_jog", 50);
        SE3_jog_publisher_ = this->create_publisher<control_msgs::msg::JointJog>("SE3_jog", 50);
        effort_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("effort_markers", 10);
        external_wrench_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("external_wrench_marker", 10);

        // Subscribers
        fdbk_joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "fdbk_joint_states", 10, std::bind(&MagnetArm::fdbk_joint_states_callback, this, std::placeholders::_1));
        ee_wrench_gravity_compensated_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "ee_wrench_gravity_compensated", 10, std::bind(&MagnetArm::ee_wrench_gravity_compensated_callback, this, std::placeholders::_1));

        // Action Client
        arm_motion_client_ = rclcpp_action::create_client<hebi_msgs::action::ArmMotion>(this, "arm_motion");

        RCLCPP_INFO(this->get_logger(), "Initializing arms");

        // Initialize Message Vectors

        // Initialize position, velocity, and effort feedbacl for all joints
        feedback_.name = joint_names_;
        feedback_.position = home_position_;

        // Initialize marker for visualizing effort
        visualization_msgs::msg::Marker effort_marker_;

        // Initialize marker array for visualizing external Wrench
        // ID 0 is force and ID 1 is torque
        external_wrench_marker_.markers.push_back(visualization_msgs::msg::Marker{});
        external_wrench_marker_.markers.push_back(visualization_msgs::msg::Marker{});

        // Force
        external_wrench_marker_.markers.at(0).header.frame_id = "base_link";
        external_wrench_marker_.markers.at(0).header.stamp = get_clock()->now();
        external_wrench_marker_.markers.at(0).id = 0;
        external_wrench_marker_.markers.at(0).type = visualization_msgs::msg::Marker::ARROW;
        external_wrench_marker_.markers.at(0).action = visualization_msgs::msg::Marker::ADD;
        external_wrench_marker_.markers.at(0).scale.x = arrow_scale;   
        external_wrench_marker_.markers.at(0).scale.y = arrow_scale;   
        external_wrench_marker_.markers.at(0).scale.z = arrow_scale;        
        external_wrench_marker_.markers.at(0).color.r = 0.5f;
        external_wrench_marker_.markers.at(0).color.g = 0.5f;
        external_wrench_marker_.markers.at(0).color.b = 1.0f;
        external_wrench_marker_.markers.at(0).color.a = 1.0;

        // Torque
        external_wrench_marker_.markers.at(1).header.frame_id = "base_link";
        external_wrench_marker_.markers.at(1).header.stamp = get_clock()->now();
        external_wrench_marker_.markers.at(1).id = 1;
        external_wrench_marker_.markers.at(1).type = visualization_msgs::msg::Marker::ARROW;
        external_wrench_marker_.markers.at(1).action = visualization_msgs::msg::Marker::DELETE; 
        external_wrench_marker_.markers.at(1).scale.x = 0.0;   
        external_wrench_marker_.markers.at(1).scale.y = 0.0;   
        external_wrench_marker_.markers.at(1).scale.z = 0.0;        
        external_wrench_marker_.markers.at(1).color.r = 0.5f;
        external_wrench_marker_.markers.at(1).color.g = 1.0f;
        external_wrench_marker_.markers.at(1).color.b = 0.5f;
        external_wrench_marker_.markers.at(1).color.a = 1.0;


        // Initialize all joint frames with respect to the base frame
        geometry_msgs::msg::TransformStamped joint_tf_{};

        // For every joint
        for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx)
        {
            // Initialize Feedback Messages
            feedback_.velocity.push_back(0.0);
            feedback_.effort.push_back(0.0);

            // Initialize TransformStamped Messages
            joint_tfs_.push_back(joint_tf_);

            // Initialize Marker Messages
            effort_marker_.header.frame_id = "base_link";
            effort_marker_.header.stamp = get_clock()->now();
            effort_marker_.id = joint_idx;
            effort_marker_.type = visualization_msgs::msg::Marker::ARROW;
            effort_marker_.action = visualization_msgs::msg::Marker::ADD;
            effort_marker_.scale.x = arrow_scale;   
            effort_marker_.scale.y = arrow_scale;   
            effort_marker_.scale.z = arrow_scale;        
            effort_marker_.color.r = 1.0f;
            effort_marker_.color.g = 0.8f;
            effort_marker_.color.b = 0.0f;
            effort_marker_.color.a = 1.0;

            effort_markers_.markers.push_back(effort_marker_);
        }

        RCLCPP_INFO(this->get_logger(), "Started Magnetic Arm Demo");
    }

private:
  
    // Create Objects
    double rate = 200.0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr cartesian_jog_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr SE3_jog_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr effort_markers_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr external_wrench_marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr fdbk_joint_states_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ee_wrench_gravity_compensated_subscriber_;
    rclcpp_action::Client<hebi_msgs::action::ArmMotion>::SharedPtr arm_motion_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Initialize state variables
    sensor_msgs::msg::JointState feedback_;
    std::vector<geometry_msgs::msg::TransformStamped> joint_tfs_;
    visualization_msgs::msg::MarkerArray effort_markers_;
    visualization_msgs::msg::MarkerArray external_wrench_marker_;
    geometry_msgs::msg::Wrench external_wrench_;

    std::vector<std::string> joint_names_;
    std::vector<double> home_position_;

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

    // Flags
    bool acting_flag_; // Indicates that an action is being executed

    // Width of effort markers
    double arrow_scale = 0.02;

    // Width of wrench marker
    double wrench_scale = 0.01;

    void fdbk_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update the feedback
        feedback_ = *msg;
    }

    void ee_wrench_gravity_compensated_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        // Update the external wrench measurement
        external_wrench_.force.x = -((*msg).force.x);
        external_wrench_.force.y = -((*msg).force.y);
        external_wrench_.force.z = -((*msg).force.z);
        external_wrench_.torque.x = -((*msg).torque.x);
        external_wrench_.torque.y = -((*msg).torque.y);
        external_wrench_.torque.z = -((*msg).torque.z);
    }

    void publish_markers()
    {
        // For every joint
        for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx)
        {
            try {
                joint_tfs_.at(joint_idx) = tf_buffer_->lookupTransform("base_link", joint_names_.at(joint_idx) + "/INPUT_INTERFACE", tf2::TimePointZero);
                RCLCPP_DEBUG(this->get_logger(), "Transform: %f, %f, %f",
                    joint_tfs_.at(joint_idx).transform.translation.x,
                    joint_tfs_.at(joint_idx).transform.translation.y,
                    joint_tfs_.at(joint_idx).transform.translation.z);
            }
            catch (tf2::TransformException & ex) {
                RCLCPP_DEBUG(this->get_logger(), "Could not transform: %s", ex.what());
            }

            // Construct effort marker
            effort_markers_.markers.at(joint_idx).header.stamp = get_clock()->now();
            effort_markers_.markers.at(joint_idx).pose.position.x = joint_tfs_.at(joint_idx).transform.translation.x;
            effort_markers_.markers.at(joint_idx).pose.position.y = joint_tfs_.at(joint_idx).transform.translation.y;
            effort_markers_.markers.at(joint_idx).pose.position.z = joint_tfs_.at(joint_idx).transform.translation.z;

            // Make the arrow point towards the Z axis of the joint frame
            // Combine the transform's orientation with a -90 degree rotation about the y-axis
            tf2::Quaternion joint_tf_quat, rot_y_90, result_quat;
            tf2::fromMsg(joint_tfs_.at(joint_idx).transform.rotation, joint_tf_quat);

            // Create a quaternion representing a -90 degree rotation around the y-axis
            rot_y_90.setRPY(0, -M_PI_2, 0);

            // Apply the rotation to the transform's orientation
            result_quat = joint_tf_quat * rot_y_90;

            // Use the resulting quaternion for the arrow marker's orientation
            effort_markers_.markers.at(joint_idx).pose.orientation = tf2::toMsg(result_quat);

            // Length and direction of arrow represent effort vector
            effort_markers_.markers.at(joint_idx).scale.x = arrow_scale * feedback_.effort.at(joint_idx);
        }

        // Publish Effort Markers
        effort_markers_publisher_->publish(effort_markers_);

        // Construct External Wrench Marker
        external_wrench_marker_.markers.at(0).pose.position.x = joint_tfs_.at(joint_names_.size() - 1).transform.translation.x;
        external_wrench_marker_.markers.at(0).pose.position.y = joint_tfs_.at(joint_names_.size() - 1).transform.translation.y;
        external_wrench_marker_.markers.at(0).pose.position.z = joint_tfs_.at(joint_names_.size() - 1).transform.translation.z;

        Eigen::Vector3d external_wrench_vec(external_wrench_.force.x, external_wrench_.force.y, external_wrench_.force.z);

        // Set marker length to be proportional to the wrench
        external_wrench_marker_.markers.at(0).scale.x = external_wrench_vec.norm() * wrench_scale; // Scale based on vector magnitude
        external_wrench_marker_.markers.at(0).scale.y = wrench_scale; // Head diameter
        external_wrench_marker_.markers.at(0).scale.z = wrench_scale; // Head length

        // Make marker point in the direction of the wrench
        external_wrench_vec.normalize();
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), external_wrench_vec);

        // Set the orientation of the marker
        external_wrench_marker_.markers.at(0).pose.orientation.x = quat.x();
        external_wrench_marker_.markers.at(0).pose.orientation.y = quat.y();
        external_wrench_marker_.markers.at(0).pose.orientation.z = quat.z();
        external_wrench_marker_.markers.at(0).pose.orientation.w = quat.w();

        external_wrench_marker_.markers.at(0).header.stamp = get_clock()->now();

        // Publish External Wrench Marker
        external_wrench_marker_publisher_->publish(external_wrench_marker_);

        RCLCPP_INFO(this->get_logger(), "External Force: %f, %f, %f",
                    external_wrench_.force.x,
                    external_wrench_.force.y,
                    external_wrench_.force.z);
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

    void SE3_jog_pub()
    {
        // Initialize SE(3) Jog Message
        control_msgs::msg::JointJog SE3_jog_msg;

        SE3_jog_msg.header.stamp = get_clock()->now();
        SE3_jog_msg.header.frame_id = "";
        SE3_jog_msg.duration = 1.0/rate; // seconds

        SE3_jog_msg.joint_names = {"roll", "pitch", "yaw", "x", "y", "z"};

        // Displacements
        for (size_t dof_idx = 0; dof_idx < SE3_jog_msg.joint_names.size(); dof_idx++)
        {
            SE3_jog_msg.displacements.push_back(0.0);
        }
        SE3_jog_msg.displacements.at(0) = roll_vel_;
        SE3_jog_msg.displacements.at(1) = pitch_vel_;
        SE3_jog_msg.displacements.at(2) = yaw_vel_;
        SE3_jog_msg.displacements.at(3) = x_vel_;
        SE3_jog_msg.displacements.at(4) = y_vel_;
        SE3_jog_msg.displacements.at(5) = z_vel_;

        // Velocities should be the same dimension as number of dof
        for (size_t dof_idx = 0; dof_idx < SE3_jog_msg.joint_names.size(); dof_idx++)
        {
            SE3_jog_msg.velocities.push_back(0.0);
        }
        SE3_jog_msg.velocities.at(0) = roll_vel_;
        SE3_jog_msg.velocities.at(1) = pitch_vel_;
        SE3_jog_msg.velocities.at(2) = yaw_vel_;
        SE3_jog_msg.velocities.at(3) = x_vel_;
        SE3_jog_msg.velocities.at(4) = y_vel_;
        SE3_jog_msg.velocities.at(5) = z_vel_;

        SE3_jog_publisher_->publish(SE3_jog_msg);
    }

    void go_home()
    {
        // Check if action server is ready
        if (!this->arm_motion_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }
        else {
            RCLCPP_DEBUG(this->get_logger(), "Action server ready");
        }

        auto goal_msg = hebi_msgs::action::ArmMotion::Goal();
        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions = home_position_;
        point.velocities = {0, 0, 0, 0, 0, 0};
        point.accelerations = {0, 0, 0, 0, 0, 0};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        goal_msg.waypoints.points.push_back(point);
        goal_msg.use_wp_times = true;
        goal_msg.wp_type = hebi_msgs::action::ArmMotion::Goal::JOINT_SPACE;

        RCLCPP_DEBUG(this->get_logger(), "Sending waypoints");

        auto send_goal_options = rclcpp_action::Client<hebi_msgs::action::ArmMotion>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MagnetArm::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&MagnetArm::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&MagnetArm::result_callback, this, std::placeholders::_1);
        
        this->arm_motion_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<hebi_msgs::action::ArmMotion>::SharedPtr & goal_handle) 
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_DEBUG(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<hebi_msgs::action::ArmMotion>::SharedPtr, const std::shared_ptr<const hebi_msgs::action::ArmMotion::Feedback> feedback) 
    {
        RCLCPP_DEBUG(this->get_logger(), "Percent Complete: %f", feedback->percent_complete);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<hebi_msgs::action::ArmMotion>::WrappedResult & result) 
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");

            // Reset acting flag
            acting_flag_ = false;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");

            // Reset acting flag
            acting_flag_ = false;
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");

            // Reset acting flag
            acting_flag_ = false;
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Motion Completed");

        // Reset acting flag
        acting_flag_ = false;
        return;
    }

    void timer_callback()
    {   
        // Re-home if START Button is pressed
        // if (controller_state_.buttons.at(7) == 1 && !acting_flag_)
        // {
        //     go_home();
        //     acting_flag_ = true; // Ensure's that the action is called only once when the START Button is pressed
        // }

        // Publish Jog Messages accordingly
        // Publish SE(3) jog messages only when there are enough degrees of freedom
        // if (joint_names_.size() >= 6)
        // {
        //     SE3_jog_pub();
        // }
        // else
        // {
        //     cartesian_jog_pub();
        // }

        publish_markers();
    }
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::MagnetArm>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}