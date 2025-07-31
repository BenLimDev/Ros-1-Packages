#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class RobotStopDetector:
    def __init__(self):
        # Remove rospy.init_node() from here - it should be initialized by the main node
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Parameters
        self.position_threshold = 0.01  # meters
        self.orientation_threshold = 0.05  # radians
        self.time_threshold = rospy.Duration(2)  # seconds
        
        # Variables to store previous state
        self.last_position = None
        self.last_orientation = None
        self.last_movement_time = rospy.Time.now()
        self.was_stopped = False  # Track if robot was stopped in previous check
        
        # Rate for main loop
        self.rate = rospy.Rate(10)  # 10Hz

    def check_stopped(self):
        try:
            # Get current transform
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            current_position = (transform.transform.translation.x,
                              transform.transform.translation.y)
            quat = transform.transform.rotation
            current_orientation = tf.transformations.euler_from_quaternion(
                [quat.x, quat.y, quat.z, quat.w])[2]  # yaw only
            
            # For first reading
            if self.last_position is None:
                self.last_position = current_position
                self.last_orientation = current_orientation
                return False
            
            # Calculate differences
            dx = abs(current_position[0] - self.last_position[0])
            dy = abs(current_position[1] - self.last_position[1])
            dtheta = abs(current_orientation - self.last_orientation)
            
            # Check if movement exceeds thresholds
            if (dx > self.position_threshold or
                dy > self.position_threshold or
                dtheta > self.orientation_threshold):
                self.last_movement_time = rospy.Time.now()
                self.last_position = current_position
                self.last_orientation = current_orientation
                self.was_stopped = False  # Robot is now moving
                return False
            else:
                # Check if we've been still for longer than time threshold
                is_currently_stopped = (rospy.Time.now() - self.last_movement_time) > self.time_threshold
                return is_currently_stopped
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return False

    def just_stopped(self):
        """Returns True only when robot has just stopped (transition from moving to stopped)"""
        is_stopped = self.check_stopped()
        
        # Check if this is a new stop event (wasn't stopped before, but is stopped now)
        just_stopped = is_stopped and not self.was_stopped
        
        # Update the previous state
        self.was_stopped = is_stopped
        
        return just_stopped

    def run(self):
        while not rospy.is_shutdown():
            is_stopped = self.check_stopped()
            if is_stopped:
                rospy.loginfo("Robot has stopped!")
            else:
                rospy.loginfo("Robot is moving...")
            self.rate.sleep()

# Standalone node functionality (only used when running this file directly)
if __name__ == '__main__':
    try:
        rospy.init_node('robot_stop_detector', anonymous=True)
        detector = RobotStopDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass