#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String


# Import MAP_LINES from the config file
from map_lines_config import MAP_LINES

# Import the RobotStopDetector class from check_moving.py
from check_moving import RobotStopDetector

# Direction visualization parameters
MARKER_LENGTH = 3.0      # 1 meter long direction marker
MARKER_WIDTH = 0.1       # 10cm wide
LINE_WIDTH = 0.05        # 5cm wide map lines
MARKER_HEIGHT = 0.1      # 10cm tall (for visibility) - Fixed: was 0
LINE_COLOR = (0, 0, 1)   # Blue for map lines (RGB)

class TTSNode:
    def __init__(self):
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
    
    def announce_status(self, message):
        msg = String()
        msg.data = message
        self.tts_pub.publish(msg)

class RobotDirectionMarker:
    def __init__(self):
        rospy.init_node('robot_direction_marker')
        
        # Store configuration
        self.map_lines = MAP_LINES
        self.marker_params = {
            'length': MARKER_LENGTH,
            'width': MARKER_WIDTH,
            'height': MARKER_HEIGHT
        }
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Setup publishers
        self.setup_publishers()
        self.tts_node = TTSNode()  # Initialize TTS system
        
        # Map metadata
        self.map_info = None
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_callback)
        
        # Start update timer
        rospy.Timer(rospy.Duration(0.1), self.update_marker)

        # Initialize the stop detector
        self.stop_detector = RobotStopDetector()

    def setup_publishers(self):
        """Initialize ROS publishers"""
        self.marker_pub = rospy.Publisher('/robot_direction_marker', Marker, queue_size=10, latch=True)
        self.line_pub = rospy.Publisher('/map_lines', Marker, queue_size=10, latch=True)

    def map_callback(self, msg):
        """Handle map metadata updates"""
        self.map_info = msg
        self.visualize_map_lines()

    def pixel_to_world(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates"""
        if not self.map_info:
            return None
        world_x = self.map_info.origin.position.x + (pixel_x * self.map_info.resolution)
        world_y = self.map_info.origin.position.y + ((self.map_info.height - pixel_y) * self.map_info.resolution)
        return (world_x, world_y)

    def visualize_map_lines(self):
        """Visualize all configured map lines with names"""
        if not self.map_info:
            return
            
        # Create a marker for all lines
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "map_lines"
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = LINE_WIDTH
        line_marker.color.r = LINE_COLOR[0]
        line_marker.color.g = LINE_COLOR[1]
        line_marker.color.b = LINE_COLOR[2]
        line_marker.color.a = 1.0
        line_marker.lifetime = rospy.Duration(0)
        
        # FIXED: Actually add points to the line marker
        line_marker.points = []
        for line in self.map_lines:
            # Convert pixel coordinates to world coordinates
            start_world = self.pixel_to_world(*line['points'][0])
            end_world = self.pixel_to_world(*line['points'][1])
            
            if start_world and end_world:
                # Add start point
                start_point = Point()
                start_point.x = start_world[0]
                start_point.y = start_world[1]
                start_point.z = 0.0
                line_marker.points.append(start_point)
                
                # Add end point
                end_point = Point()
                end_point.x = end_world[0]
                end_point.y = end_world[1]
                end_point.z = 0.0
                line_marker.points.append(end_point)
        
        # Only publish if we have points
        if line_marker.points:
            self.line_pub.publish(line_marker)

    def line_intersection(self, line1, line2):
        """Check if two line segments intersect"""
        x1, y1 = line1[0]
        x2, y2 = line1[1]
        x3, y3 = line2[0]
        x4, y4 = line2[1]
        
        denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
        if denom == 0:  # lines are parallel
            return False
            
        ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
        ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
        
        return 0 <= ua <= 1 and 0 <= ub <= 1

    def update_marker(self, event):
        """Update the robot direction marker"""
        try:
            if not self.map_info:
                return
                
            # Get robot pose
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            quat = transform.transform.rotation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            
            # Create and publish marker
            self.publish_direction_marker(x, y, yaw)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5.0, "TF error: %s", str(e))

    def publish_direction_marker(self, x, y, yaw):
        """Create and publish the direction marker"""
        # Calculate marker end point
        end_x = x + math.cos(yaw) * self.marker_params['length']
        end_y = y + math.sin(yaw) * self.marker_params['length']
        
        # Check intersections with all lines
        intersecting = False
        intersecting_line_name = None
        marker_line = ((x, y), (end_x, end_y))
        
        for line in self.map_lines:
            world_line = (
                self.pixel_to_world(*line['points'][0]),
                self.pixel_to_world(*line['points'][1])
            )
            
            if None in world_line:
                continue
                
            if self.line_intersection(marker_line, world_line):
                intersecting = True
                intersecting_line_name = line['name']
                break
        
        # Create marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_direction"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self.marker_params['height']
        
        # FIXED: Properly initialize quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        marker.scale.x = self.marker_params['length']
        marker.scale.y = self.marker_params['width']
        marker.scale.z = self.marker_params['height']
        
        # Set color based on intersection
        marker.color.r = 1.0  # Red by default
        marker.color.g = 0.0  # FIXED: Initialize green component
        marker.color.b = 0.0  # FIXED: Initialize blue component
        marker.color.a = 1.0
        

        # Color and TTS logic
        if intersecting:
            # Yellow marker for intersections
            marker.color.g = 1.0 
            
            if self.stop_detector.just_stopped():
                rospy.loginfo_throttle(1.0, f"Facing {intersecting_line_name}!")
                self.tts_node.announce_status(f"Facing {intersecting_line_name}!")
        else:
            # Red marker for no intersections
            marker.color.g = 0
            
            if self.stop_detector.just_stopped():
                rospy.loginfo_throttle(2.0, "No intersection detected")
                self.tts_node.announce_status("Not close to anything!")
        
        
        self.marker_pub.publish(marker)
        

if __name__ == '__main__':
    try:
        RobotDirectionMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

