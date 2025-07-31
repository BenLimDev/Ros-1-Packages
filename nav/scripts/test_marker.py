#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

class RobotDirectionMarker:
    def __init__(self):
        rospy.init_node('robot_direction_marker')
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Marker publisher (with latching)
        self.marker_pub = rospy.Publisher('/robot_direction_marker', Marker, queue_size=10, latch=True)
        
        rospy.loginfo("Waiting for transform and initializing marker...")
        self.wait_for_transform()
        
        # Start continuous updates
        rospy.Timer(rospy.Duration(0.1), self.update_marker)

    def wait_for_transform(self):
        while not rospy.is_shutdown():
            try:
                # Check transform exists
                self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time(0))
                # Publish initial marker
                self.update_marker(None)
                rospy.loginfo("TF data available and marker initialized!")
                return
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                rospy.sleep(0.1)
                continue

    def update_marker(self, event):
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time(0))
            
            # Extract position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            quat = transform.transform.rotation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            
            # Create marker
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_direction"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set marker properties
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))
            
            marker.scale.x = 1.0
            marker.scale.y = 0.1   # Width
            marker.scale.z = 0.1   # Height
            
            marker.color.r = 1.0   # Red
            marker.color.a = 1.0   # Fully opaque
            
            marker.lifetime = rospy.Duration(0.5)  # Auto-disappear if not updated
            
            # Publish marker
            self.marker_pub.publish(marker)
            
            # Log position/yaw (throttled to 1Hz)
            rospy.loginfo_throttle(1.0, 
                f"Published marker at [x:{x:.2f}, y:{y:.2f}] | "
                f"Yaw: {math.degrees(yaw):.1f}°"
            )
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5.0, "TF error: %s", str(e))

if __name__ == '__main__':
    try:
        RobotDirectionMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass