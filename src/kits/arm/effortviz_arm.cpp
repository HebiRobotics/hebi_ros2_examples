#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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

class EffortVizArm : public rclcpp::Node {
public:
  
  EffortVizArm() : Node("effortviz_arm") {

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
        std::bind(&EffortVizArm::timer_callback, this));

        // Publishers
        effort_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("effort_markers", 10);
        external_wrench_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("external_wrench_markers", 10);

        // Subscribers
        fdbk_joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "fdbk_joint_states", 10, std::bind(&EffortVizArm::fdbk_joint_states_callback, this, std::placeholders::_1));
        ee_wrench_gravity_compensated_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "ee_wrench_gravity_compensated", 10, std::bind(&EffortVizArm::ee_wrench_gravity_compensated_callback, this, std::placeholders::_1));

        // Parameters Client
        // parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "arm_node");

        // Initialize Messages

        // Initialize marker for visualizing effort
        visualization_msgs::msg::Marker effort_marker_;

        // Initialize marker array for visualizing external wrench
        // ID 0 is force and ID 1 is torque
        external_wrench_markers_.markers.push_back(visualization_msgs::msg::Marker{});
        external_wrench_markers_.markers.push_back(visualization_msgs::msg::Marker{});

        // Force Marker
        external_wrench_markers_.markers.at(0).header.frame_id = "base_link";
        external_wrench_markers_.markers.at(0).header.stamp = get_clock()->now();
        external_wrench_markers_.markers.at(0).id = 0;
        external_wrench_markers_.markers.at(0).type = visualization_msgs::msg::Marker::ARROW;
        external_wrench_markers_.markers.at(0).action = visualization_msgs::msg::Marker::ADD;
        external_wrench_markers_.markers.at(0).scale.x = wrench_scale_;   
        external_wrench_markers_.markers.at(0).scale.y = wrench_scale_;   
        external_wrench_markers_.markers.at(0).scale.z = wrench_scale_;        
        external_wrench_markers_.markers.at(0).color.r = 0.5f;
        external_wrench_markers_.markers.at(0).color.g = 0.5f;
        external_wrench_markers_.markers.at(0).color.b = 1.0f;
        external_wrench_markers_.markers.at(0).color.a = 1.0;

        // Torque Marker
        external_wrench_markers_.markers.at(1).header.frame_id = "base_link";
        external_wrench_markers_.markers.at(1).header.stamp = get_clock()->now();
        external_wrench_markers_.markers.at(1).id = 1;
        external_wrench_markers_.markers.at(1).type = visualization_msgs::msg::Marker::ARROW;
        external_wrench_markers_.markers.at(1).action = visualization_msgs::msg::Marker::DELETE; 
        external_wrench_markers_.markers.at(1).scale.x = wrench_scale_;   
        external_wrench_markers_.markers.at(1).scale.y = wrench_scale_;   
        external_wrench_markers_.markers.at(1).scale.z = wrench_scale_;        
        external_wrench_markers_.markers.at(1).color.r = 0.5f;
        external_wrench_markers_.markers.at(1).color.g = 1.0f;
        external_wrench_markers_.markers.at(1).color.b = 0.5f;
        external_wrench_markers_.markers.at(1).color.a = 1.0;

        // Initialize all joint frames with respect to the base frame
        geometry_msgs::msg::TransformStamped joint_tf_{};

        // Initialize position, velocity, and effort feedback for all joints
        feedback_.name = joint_names_;
        feedback_.position = home_position_;

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
            effort_marker_.scale.x = effort_scale_;   
            effort_marker_.scale.y = effort_scale_;   
            effort_marker_.scale.z = effort_scale_;        
            effort_marker_.color.r = 1.0f;
            effort_marker_.color.g = 0.8f;
            effort_marker_.color.b = 0.0f;
            effort_marker_.color.a = 1.0;

            effort_markers_.markers.push_back(effort_marker_);
        }

        RCLCPP_INFO(this->get_logger(), "Started Effort Visualization Demo");
    }

private:
  
    // Create Objects
    double rate = 200.0;
    double dt_ = 1/rate;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr effort_markers_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr external_wrench_markers_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr fdbk_joint_states_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ee_wrench_gravity_compensated_subscriber_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Create state variables
    sensor_msgs::msg::JointState feedback_;
    std::vector<geometry_msgs::msg::TransformStamped> joint_tfs_;
    visualization_msgs::msg::MarkerArray effort_markers_;
    visualization_msgs::msg::MarkerArray external_wrench_markers_;
    geometry_msgs::msg::Wrench external_wrench_;

    // Create Arm Parameters
    std::vector<std::string> joint_names_;
    std::vector<double> home_position_;

    // Scaling factors for markers
    double effort_scale_ = 0.02;
    double wrench_scale_ = 0.01;

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
        // EFFORT MARKERS
        
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
            effort_markers_.markers.at(joint_idx).scale.x = effort_scale_ * feedback_.effort.at(joint_idx);
        }

        // Publish Effort Markers
        effort_markers_publisher_->publish(effort_markers_);

        // EXTERNAL WRENCH MARKERS

        external_wrench_markers_.markers.at(0).header.stamp = get_clock()->now(); // Force
        external_wrench_markers_.markers.at(1).header.stamp = get_clock()->now(); // Torque

        // Define marker position at end-effector
        // Force
        external_wrench_markers_.markers.at(0).pose.position.x = joint_tfs_.at(joint_names_.size() - 1).transform.translation.x;
        external_wrench_markers_.markers.at(0).pose.position.y = joint_tfs_.at(joint_names_.size() - 1).transform.translation.y;
        external_wrench_markers_.markers.at(0).pose.position.z = joint_tfs_.at(joint_names_.size() - 1).transform.translation.z;
        // Torque
        external_wrench_markers_.markers.at(1).pose.position.x = joint_tfs_.at(joint_names_.size() - 1).transform.translation.x;
        external_wrench_markers_.markers.at(1).pose.position.y = joint_tfs_.at(joint_names_.size() - 1).transform.translation.y;
        external_wrench_markers_.markers.at(1).pose.position.z = joint_tfs_.at(joint_names_.size() - 1).transform.translation.z;

        Eigen::Vector3d external_force_vec(external_wrench_.force.x, external_wrench_.force.y, external_wrench_.force.z),
                        external_torque_vec(external_wrench_.torque.x, external_wrench_.torque.y, external_wrench_.torque.z);

        // Set force marker length to be proportional to the force
        external_wrench_markers_.markers.at(0).scale.x = external_force_vec.norm() * wrench_scale_; // Scale based on force magnitude
        // Set torque marker length to be proportional to the torque
        external_wrench_markers_.markers.at(1).scale.x = external_torque_vec.norm() * wrench_scale_; // Scale based on torque magnitude

        // Orient markers based on wrench values
        // Force
        external_force_vec.normalize();
        Eigen::Quaterniond quat_force = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), external_force_vec);
        // Torque
        external_torque_vec.normalize();
        Eigen::Quaterniond quat_torque = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), external_torque_vec);

        // Set the orientation
        // Force
        external_wrench_markers_.markers.at(0).pose.orientation.x = quat_force.x();
        external_wrench_markers_.markers.at(0).pose.orientation.y = quat_force.y();
        external_wrench_markers_.markers.at(0).pose.orientation.z = quat_force.z();
        external_wrench_markers_.markers.at(0).pose.orientation.w = quat_force.w();
        // Torque
        external_wrench_markers_.markers.at(1).pose.orientation.x = quat_torque.x();
        external_wrench_markers_.markers.at(1).pose.orientation.y = quat_torque.y();
        external_wrench_markers_.markers.at(1).pose.orientation.z = quat_torque.z();
        external_wrench_markers_.markers.at(1).pose.orientation.w = quat_torque.w();

        // Publish External Wrench Markers
        external_wrench_markers_publisher_->publish(external_wrench_markers_);
    }

    void timer_callback()
    {   
        // Publish Markers for visualization at a fixed rate
        publish_markers();
    }
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hebi::ros::EffortVizArm>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}