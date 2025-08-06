#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hebi_msgs.msg import SE3Jog
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import time
import threading
from dataclasses import dataclass, field
from pyOpenHaptics.hd_device import HapticDevice
import pyOpenHaptics.hd as hd
from pyOpenHaptics.hd_callback import hd_callback
from scipy.spatial.transform import Rotation as R

@dataclass
class DeviceState:
    stylus_button: bool = False
    last_stylus_button: bool = False  # Track previous stylus button state for toggle detection
    extra_button: bool = False
    last_extra_button: bool = False  # Track previous extra button state for home detection
    jog_enabled: bool = False  # Toggle state for publishing jog commands
    transform: np.ndarray = field(default_factory=lambda: np.eye(4))
    force: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # Force command [x, y, z] in Newtons
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(6))  # Joint state for 6DOF haptic device

# Global variables
device_state = None
device_lock = threading.Lock()  # Lock for thread-safe access to device state
haptic_device = None  # Global reference to haptic device for proper cleanup
shutdown_event = threading.Event()  # Event to signal shutdown to haptic thread

@hd_callback
def hd_update():
    global device_state, device_lock
    
    if device_state is None:
        return
        
    with device_lock:
        transform = hd.get_transform()
        # Convert transform to numpy array and save to state
        device_state.transform = np.array(transform).T
        
        # Set force from device state
        hd.set_force(device_state.force)
        
        # Handle button states
        buttons = hd.get_buttons()
        device_state.stylus_button = bool(buttons & 1)  # Bit 0: stylus button
        device_state.extra_button = bool(buttons & 2)   # Bit 1: extra button

        # Set joint angles
        joints = np.array(hd.get_joints())
        gimbals = np.array(hd.get_gimbals())
        # Convert ctypes arrays to numpy arrays before concatenation
        device_state.joint_angles = np.concatenate((joints, gimbals))

class HapticROSNode(Node):
    """ROS2 node that publishes SE3 jog commands from haptic device"""

    def __init__(self):
        super().__init__('haptic_jog_node')
        
        # Declare parameters
        self.declare_parameter('prefix', '/')
        self.declare_parameter('rate', 500.0)
        self.declare_parameter('jog_scale', [0.002, 0.002, 0.002, 1.0, 1.0, 1.0])  # [x, y, z, roll, pitch, yaw]
        self.declare_parameter('max_force', [4.0, 4.0, 4.0])  # Maximum force in each axis [x, y, z] in Newtons
        self.declare_parameter('tanh_constant', 1.5)  # Constant for tanh force scaling
        self.declare_parameter('dead_zone', 1.2)  # Dead zone for force feedback
        
        # Get parameters
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.dt = 1.0 / self.rate  # Time step for jog commands
        self.jog_scale = np.array(self.get_parameter('jog_scale').get_parameter_value().double_array_value)
        self.max_force = np.array(self.get_parameter('max_force').get_parameter_value().double_array_value)
        self.force_tanh_constant = self.get_parameter('tanh_constant').get_parameter_value().double_value
        self.force_dead_zone = self.get_parameter('dead_zone').get_parameter_value().double_value
        
        # Publisher for SE3 jog commands (6DOF)
        self.se3_jog_publisher = self.create_publisher(SE3Jog, f'{self.prefix}SE3_jog', 50)
        
        # Subscriber for end-effector wrench feedback
        self.ee_force_subscriber = self.create_subscription(
            Vector3Stamped,
            f'{self.prefix}ee_force',
            self.ee_force_callback,
            10
        )
        
        # Home service client
        self.home_client = self.create_client(Trigger, f'{self.prefix}home')

        # Timer for publishing
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # State variables
        self.last_haptic_position = np.zeros(3)
        self.last_haptic_rotation = np.eye(3)
        self.last_joint_angles = np.zeros(6)
        
        # SE3 velocities
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0
        self.roll_vel = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0
        
        # Low pass filter admittance
        self.low_pass_admittance = 1.0  # Adjust as needed for responsiveness
        
        self.force_scale = 20.0
        # Low pass filter for force feedback
        self.force_low_pass_admittance = 0.95  # Adjust as needed for force smoothness
        self.filtered_force = np.array([0.0, 0.0, 0.0])  # Filtered force values

        self.max_safe_joint_vel = 100.0
        
        self.get_logger().info("Started Haptic Device SE3 Jog Controller with Force Feedback")
        self.get_logger().info(f"Publishing SE3 jog commands on: {self.prefix}/SE3_jog" if self.prefix else "SE3_jog")
        self.get_logger().info(f"Subscribing to end-effector wrench on: {self.prefix}ee_wrench" if self.prefix else "ee_wrench")
        self.get_logger().info(f"Max force limits: [{self.max_force[0]:.1f}, {self.max_force[1]:.1f}, {self.max_force[2]:.1f}] N")
        self.get_logger().info("Press stylus button to toggle jog command publishing")
        self.get_logger().info("Press extra button to call home service")
    
    def cleanup(self):
        """Cleanup method to properly shutdown the node and haptic device"""
        global shutdown_event
        
        self.get_logger().info("Cleaning up haptic ROS node...")
        
        # Clear all force commands
        self.clear_device_force()
        
        # Signal haptic thread to shutdown
        shutdown_event.set()
        
        # Destroy timer
        if hasattr(self, 'timer'):
            self.timer.destroy()
        
        self.get_logger().info("Haptic ROS node cleanup complete")
    
    def extract_haptic_position_and_orientation(self):
        """Extract position and orientation from haptic device transform matrix"""
        with device_lock:
            if device_state is None:
                # Return identity if device not available
                return np.zeros(3), np.eye(3)
            transform = device_state.transform.copy()
        
        # Position (translation) from the last column with coordinate transformation
        position = np.array([-transform[2, 3], -transform[0, 3], transform[1, 3]])
        
        # Apply coordinate transformation to fix rotation mapping
        coord_transform = np.array([[0, 0, -1],
                                   [1, 0, 0],
                                   [0, -1, 0]])
        
        # Transform the rotation matrix: T * R * T^(-1)
        rotation = coord_transform @ transform[:3, :3] @ coord_transform.T
        
        return position, rotation
    
    def calculate_jog_velocities(self, haptic_position, haptic_rotation):
        """Calculate SE3 jog velocities from haptic device movement"""
        # Calculate position delta
        delta_position = (haptic_position - self.last_haptic_position) * self.jog_scale[:3]
        
        # Calculate rotation delta in local frame
        delta_rotation_matrix = self.last_haptic_rotation.T @ haptic_rotation
        delta_euler = R.from_matrix(delta_rotation_matrix).as_euler('xyz') * self.jog_scale[3:]
        
        # Apply low pass filtering
        self.x_vel += self.low_pass_admittance * (delta_position[0] - self.x_vel)
        self.y_vel += self.low_pass_admittance * (delta_position[1] - self.y_vel)
        self.z_vel += self.low_pass_admittance * (delta_position[2] - self.z_vel)
        self.roll_vel += self.low_pass_admittance * (delta_euler[0] - self.roll_vel)
        self.pitch_vel += self.low_pass_admittance * (delta_euler[1] - self.pitch_vel)
        self.yaw_vel += self.low_pass_admittance * (delta_euler[2] - self.yaw_vel)
    
    def ee_force_callback(self, msg):
        """Callback for end-effector wrench feedback
        
        Args:
            msg (Vector3Stamped): Wrench message containing force and torque data
        """
        # Extract force components from the wrench message
        force_x = -msg.vector.y * self.force_scale
        force_y = msg.vector.z * self.force_scale
        force_z = -msg.vector.x * self.force_scale

        # Apply low-pass filtering to the force feedback
        raw_force = np.array([force_x, force_y, force_z])
        self.filtered_force += self.force_low_pass_admittance * (raw_force - self.filtered_force)
        
        # Set the filtered force on the haptic device (only when jog is enabled)
        if device_state.jog_enabled:
            self.set_device_force(self.filtered_force[0], self.filtered_force[1], self.filtered_force[2])
            
            self.get_logger().debug(
                f"Raw force: [{force_x:.3f}, {force_y:.3f}, {force_z:.3f}] N, "
                f"Filtered: [{self.filtered_force[0]:.3f}, {self.filtered_force[1]:.3f}, {self.filtered_force[2]:.3f}] N"
            )
    
    def set_device_force(self, force_x=0.0, force_y=0.0, force_z=0.0):
        """Set force command for the haptic device
        
        Args:
            force_x (float): Force in X direction (Newtons)
            force_y (float): Force in Y direction (Newtons) 
            force_z (float): Force in Z direction (Newtons)
        """
        force = np.array([force_x, force_y, force_z])
        force = self.max_force / 2 * (
                    np.tanh(self.force_tanh_constant * ( force - self.force_dead_zone)) -
                    np.tanh(self.force_tanh_constant * (-force - self.force_dead_zone))
                )
        
        # Update device state with new force command (thread-safe)
        with device_lock:
            device_state.force = force.tolist()
        
        self.get_logger().debug(f"Force command set: [{force[0]:.3f}, {force[1]:.3f}, {force[2]:.3f}] N")
    
    def clear_device_force(self):
        """Clear all force commands (set to zero)"""
        with device_lock:
            device_state.force = [0.0, 0.0, 0.0]
        
        # Also reset filtered force to prevent sudden jumps when re-enabling
        self.filtered_force = np.array([0.0, 0.0, 0.0])
        
        self.get_logger().debug("Force commands cleared")
    
    def publish_se3_jog(self):
        """Publish SE3 (6DOF) jog message"""
        msg = SE3Jog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.dx = self.x_vel
        msg.dy = self.y_vel
        msg.dz = self.z_vel
        msg.droll = self.roll_vel
        msg.dpitch = self.pitch_vel
        msg.dyaw = self.yaw_vel
        
        msg.duration = 0.3
        
        self.se3_jog_publisher.publish(msg)
    
    def go_home(self):
        """Call home service"""
        request = Trigger.Request()
        
        # Check if service is available
        if not self.home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Home service not available")
            return
        
        future = self.home_client.call_async(request)
        self.get_logger().info("Home service called")
        
        # Clear force commands when homing
        self.clear_device_force()
        
        # Always disable jog after homing
        with device_lock:
            if device_state is not None and device_state.jog_enabled:
                device_state.jog_enabled = False
                print("Jog command publishing: DISABLED (after homing)")
    
    def timer_callback(self):
        """Main timer callback for publishing SE3 jog commands"""
        # Check if device state is available
        with device_lock:
            if device_state is None:
                # Device not ready or shutting down
                return
                
            extra_button = device_state.extra_button
            last_extra_button = device_state.last_extra_button
            stylus_button = device_state.stylus_button
            last_stylus_button = device_state.last_stylus_button
            jog_enabled = device_state.jog_enabled
        
        if extra_button and not last_extra_button:
            # Extra button just pressed - call home service
            self.go_home()
        
        # Handle stylus button press for toggling jog commands
        if stylus_button and not last_stylus_button:
            # Stylus button just pressed - toggle jog command publishing
            with device_lock:
                if device_state is not None:
                    device_state.jog_enabled = not device_state.jog_enabled  # Toggle jog command publishing
                    jog_enabled = device_state.jog_enabled
            print(f"Jog command publishing: {'ENABLED' if jog_enabled else 'DISABLED'}")
        
        # Update last button states and also get joint angles
        with device_lock:
            if device_state is not None:
                device_state.last_stylus_button = stylus_button
                device_state.last_extra_button = extra_button
                joint_angles = device_state.joint_angles.copy()

        # Calculate joint velocities and update last joint angles
        joint_velocities = (joint_angles - self.last_joint_angles) / self.dt
        self.last_joint_angles = joint_angles.copy()

        # SAFETY: Disable jog if joint velocities are too high
        if jog_enabled and np.any(np.abs(joint_velocities) > self.max_safe_joint_vel):
            with device_lock:
                if device_state is not None:
                    device_state.jog_enabled = False
                    jog_enabled = False
            print("Jog command publishing: DISABLED (joint velocity limit exceeded)")

        # Get current haptic transform
        haptic_position, haptic_rotation = self.extract_haptic_position_and_orientation()
        
        # Only process and publish if jog is enabled
        if jog_enabled:
            # Calculate jog velocities from haptic device movement
            self.calculate_jog_velocities(haptic_position, haptic_rotation)
            
            # Publish SE3 jog commands (6DOF)
            self.publish_se3_jog()
            
            self.get_logger().debug(
                f"SE3 Jog: [{self.x_vel:.4f}, {self.y_vel:.4f}, {self.z_vel:.4f}, "
                f"{self.roll_vel:.4f}, {self.pitch_vel:.4f}, {self.yaw_vel:.4f}]")
        else:
            # Reset velocities when disabled (but don't publish)
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.z_vel = 0.0
            self.roll_vel = 0.0
            self.pitch_vel = 0.0
            self.yaw_vel = 0.0
            
            # Clear force commands when jog is disabled
            self.clear_device_force()
        
        # Update last haptic state for next calculation
        self.last_haptic_position = haptic_position.copy()
        self.last_haptic_rotation = haptic_rotation.copy()

def haptic_thread_function():
    """Function to run haptic device in separate thread"""
    global device_state, device_lock, haptic_device, shutdown_event
    
    print("Initializing haptic device in separate thread...")
    
    try:
        device_state = DeviceState()
        haptic_device = HapticDevice(device_name="Default Device", callback=hd_update)
        time.sleep(0.2)  # Give haptic device time to initialize
        print("Haptic device initialized and running in background thread.")
        
        # Keep the haptic device running until shutdown signal
        while not shutdown_event.is_set():
            time.sleep(0.001)  # Small sleep to prevent excessive CPU usage
            
    except Exception as e:
        print(f"Haptic thread error: {e}")
    finally:
        # Clean up haptic device
        if haptic_device is not None:
            try:
                haptic_device.close()
                print("Haptic device closed.")
            except Exception as e:
                print(f"Error closing haptic device: {e}")
        
        # Clean up device state
        with device_lock:
            device_state = None

def main(args=None):
    global device_state, device_lock, shutdown_event
    
    print("Starting Haptic Device ROS2 SE3 Jog Publisher...")
    print("Move your haptic device to generate SE3 jog commands.")
    print("Press the stylus button to TOGGLE jog command publishing on/off.")
    print("Press the extra button to call home service.")
    print("Press Ctrl+C to exit.")
    
    # Reset shutdown event
    shutdown_event.clear()
    
    # Start haptic device in separate thread (not daemon to ensure proper cleanup)
    haptic_thread = threading.Thread(target=haptic_thread_function, daemon=False)
    haptic_thread.start()
    time.sleep(0.5)  # Give haptic thread time to initialize
    
    # Check if haptic device was initialized successfully
    if device_state is None:
        print("Failed to initialize haptic device. Exiting...")
        shutdown_event.set()
        haptic_thread.join(timeout=2.0)
        return

    # Initialize ROS2 AFTER haptic device is ready
    rclpy.init(args=args)
    
    node = None
    try:
        # Create the ROS node
        node = HapticROSNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down haptic ROS node...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup ROS node
        if node is not None:
            node.cleanup()
            node.destroy_node()
        
        # Shutdown ROS
        rclpy.shutdown()
        print("ROS node closed.")
        
        # Signal haptic thread to shutdown and wait for it
        shutdown_event.set()
        print("Waiting for haptic thread to finish...")
        haptic_thread.join(timeout=5.0)  # Wait up to 5 seconds for clean shutdown
        
        if haptic_thread.is_alive():
            print("Warning: Haptic thread did not shutdown cleanly")
        else:
            print("Haptic thread closed successfully.")
        
        print("Application shutdown complete.")

if __name__ == "__main__":
    main()
