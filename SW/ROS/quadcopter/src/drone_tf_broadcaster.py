#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
import math

class DroneTFBroadcaster:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('drone_tf_broadcaster', anonymous=True)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Drone parameters
        self.arm_length = 0.11      # Distance from center to arm tip (meters)
        self.arm_height = 0.00     # Height of arm above base_link (meters)
        self.prop_offset = 0.001    # Height of propeller above arm (meters)
        self.prop_spin_rate = 10.0 # Propeller rotation rate (rad/s)
        self.throttle_threshold = 1040  # Minimum throttle value for propeller motion
        
        # Drone position in fixed frame
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.0         # Hover at 1m height
        
        # Drone orientation from IMU (initialized to level)
        self.drone_imu_x = 0
        self.drone_imu_y = 0
        self.drone_imu_z = 0
        self.drone_imu_w = -1
        
        # Throttle values for each ESC
        self.throttle_rr = 0  # Rear Right
        self.throttle_rl = 0  # Rear Left
        self.throttle_fl = 0  # Front Left
        self.throttle_fr = 0  # Front Right
        
        # Propeller rotation states
        self.prop_rotations = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }
        
        # Time tracking
        self.start_time = rospy.Time.now()
        self.last_time = self.start_time
        
        # Initialize subscribers
        self.init_subscribers()
        
        # Timer to broadcast transforms at 50Hz
        self.timer = rospy.Timer(rospy.Duration(0.02), self.broadcast_transforms)
        
        rospy.loginfo("Drone TF Broadcaster with IMU and Throttle initialized")

    def init_subscribers(self):
        """Initialize ROS subscribers for throttle and IMU data"""
        
        # Throttle subscribers
        rospy.Subscriber('throttle/esc_RR', Int16, self.throttle_rr_callback)
        rospy.Subscriber('throttle/esc_RL', Int16, self.throttle_rl_callback)
        rospy.Subscriber('throttle/esc_FL', Int16, self.throttle_fl_callback)
        rospy.Subscriber('throttle/esc_FR', Int16, self.throttle_fr_callback)
        
        # IMU subscriber
        rospy.Subscriber('imu/data_raw', Imu, self.imu_callback)
        
        rospy.loginfo("Subscribers initialized for throttle and IMU data")

    def throttle_rr_callback(self, msg):
        """Callback for rear right throttle"""
        self.throttle_rr = msg.data

    def throttle_rl_callback(self, msg):
        """Callback for rear left throttle"""
        self.throttle_rl = msg.data

    def throttle_fl_callback(self, msg):
        """Callback for front left throttle"""
        self.throttle_fl = msg.data

    def throttle_fr_callback(self, msg):
        """Callback for front right throttle"""
        self.throttle_fr = msg.data

    def imu_callback(self, msg):
        """Callback for IMU data - extract roll, pitch, yaw from quaternion"""
        try:
            self.drone_imu_x = msg.orientation.x
            self.drone_imu_y = msg.orientation.y
            self.drone_imu_z = msg.orientation.z
            self.drone_imu_w = msg.orientation.w
            

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")

    def update_propeller_rotations(self, dt):
        """Update propeller rotations based on throttle values"""
        
        # Map throttle values to rotation speeds
        # Only rotate if throttle > threshold
        throttle_configs = [
            ('front_left', self.throttle_fl, 1),   # Clockwise
            ('front_right', self.throttle_fr, -1), # Counter-clockwise
            ('rear_left', self.throttle_rl, -1),   # Counter-clockwise
            ('rear_right', self.throttle_rr, 1)    # Clockwise
        ]
        
        for prop_name, throttle_val, direction in throttle_configs:
            if throttle_val > self.throttle_threshold:
                # Map throttle (1040-2000) to rotation speed (0-20 rad/s)
                normalized_throttle = max(0, min(1, (throttle_val - self.throttle_threshold) / (2000 - self.throttle_threshold)))
                rotation_speed = normalized_throttle * self.prop_spin_rate * 2  # Max 20 rad/s
                
                # Update rotation angle
                self.prop_rotations[prop_name] += direction * rotation_speed * dt
                
                # Keep angle in reasonable range
                self.prop_rotations[prop_name] = self.prop_rotations[prop_name] % (2 * math.pi)
            # If throttle <= threshold, propeller stops (maintains current position)

    def broadcast_transforms(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Update propeller rotations based on throttle values
        self.update_propeller_rotations(dt)
        
        # Broadcast fixed_frame to base_link transform
        self.broadcast_drone_transform(current_time)
        
        # Broadcast arm transforms
        self.broadcast_arm_transforms(current_time)
        
        # Broadcast propeller transforms (with throttle-controlled rotation)
        self.broadcast_propeller_transforms(current_time)

    def broadcast_drone_transform(self, timestamp):
        """Broadcast transform from fixed_frame to base_link using IMU orientation"""
        transform = geometry_msgs.msg.TransformStamped()
        
        transform.header.stamp = timestamp
        transform.header.frame_id = "fixed_frame"
        transform.child_frame_id = "base_link"
        
        # Position (could be updated based on flight controller or GPS data)
        transform.transform.translation.x = self.drone_x
        transform.transform.translation.y = self.drone_y
        transform.transform.translation.z = self.drone_z
        
        # Orientation from IMU data
        transform.transform.rotation.x = self.drone_imu_x
        transform.transform.rotation.y = self.drone_imu_y
        transform.transform.rotation.z = self.drone_imu_z
        transform.transform.rotation.w = self.drone_imu_w
        
        self.tf_broadcaster.sendTransform(transform)

    def broadcast_arm_transforms(self, timestamp):
        """Broadcast transforms from base_link to each arm"""
        
        # Arm positions and orientations (45-degree spacing)
        arm_configs = [
            ("arm_front_left", math.pi/4, "front_left"),
            ("arm_front_right", -math.pi/4, "front_right"), 
            ("arm_rear_left", 3*math.pi/4, "rear_left"),
            ("arm_rear_right", -3*math.pi/4, "rear_right")
        ]
        
        for arm_name, angle, description in arm_configs:
            transform = geometry_msgs.msg.TransformStamped()
            
            transform.header.stamp = timestamp
            transform.header.frame_id = "base_link"
            transform.child_frame_id = arm_name
            
            # Position arm at specified angle
            transform.transform.translation.x = 0
            transform.transform.translation.y = 0
            transform.transform.translation.z = self.arm_height
            
            # Rotate arm to point outward
            quaternion = quaternion_from_euler(0, 0, angle)
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            
            self.tf_broadcaster.sendTransform(transform)

    def broadcast_propeller_transforms(self, timestamp):
        """Broadcast transforms from each arm to its propeller with throttle-controlled rotation"""
        
        # Propeller configurations
        prop_configs = [
            ("arm_front_left", "prop_front_left", "front_left"),
            ("arm_front_right", "prop_front_right", "front_right"),
            ("arm_rear_left", "prop_rear_left", "rear_left"),
            ("arm_rear_right", "prop_rear_right", "rear_right")
        ]
        
        for parent_frame, prop_frame, prop_key in prop_configs:
            transform = geometry_msgs.msg.TransformStamped()
            
            transform.header.stamp = timestamp
            transform.header.frame_id = parent_frame
            transform.child_frame_id = prop_frame
            
            # Position propeller at end of arm
            transform.transform.translation.x = self.arm_length
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = self.prop_offset
            
            # Rotate propeller based on throttle-controlled rotation
            rotation = self.prop_rotations[prop_key]
            quaternion = quaternion_from_euler(0, 0, rotation)
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            
            self.tf_broadcaster.sendTransform(transform)

    def run(self):
        """Keep the node running"""
        rospy.loginfo("Drone TF Broadcaster running...")
        rospy.spin()

def main():
    try:
        broadcaster = DroneTFBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Drone TF Broadcaster shutting down...")
        pass

if __name__ == '__main__':
    main()
