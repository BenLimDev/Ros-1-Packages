#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
import yaml
import os

# import the map lines from the YAML file
def load_map_lines():
    yaml_path = os.path.join(
        os.path.dirname(__file__), '..', 'lines_params', 'lab_area.yaml'
    )
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['map_lines']

# Import the RobotStopDetector class from check_moving.py
from check_moving import RobotStopDetector

# Direction visualization parameters
MARKER_LENGTH = 3.0      # 1 meter long direction marker
MARKER_WIDTH = 0.1       # 10cm wide
LINE_WIDTH = 0.05        # 5cm wide map lines
MARKER_HEIGHT = 0.1      # 10cm tall (for visibility)
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
        self.map_lines = load_map_lines()
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
        # Publisher for robot direction arrow (single Marker)
        self.arrow_pub = rospy.Publisher('/robot_direction_marker', Marker, queue_size=10)
        # Publisher for map lines + text (MarkerArray)
        self.line_pub = rospy.Publisher('/map_lines', MarkerArray, queue_size=10, latch=True)

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
        """Visualize all configured map lines with individual colors and text labels"""
        if not self.map_info:
            return

        marker_array = MarkerArray()
        marker_id = 0

        for line in self.map_lines:
            start_world = self.pixel_to_world(*line['points'][0])
            end_world = self.pixel_to_world(*line['points'][1])
            if not (start_world and end_world):
                continue

            # --- Line marker for this line ---
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "map_lines"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.scale.x = LINE_WIDTH
            r, g, b = line.get('color', (0.0, 0.0, 1.0))  # default blue
            line_marker.color.r = r
            line_marker.color.g = g
            line_marker.color.b = b
            line_marker.color.a = 1.0
            line_marker.lifetime = rospy.Duration(0)

            start_point = Point(x=start_world[0], y=start_world[1], z=0.0)
            end_point = Point(x=end_world[0], y=end_world[1], z=0.0)
            line_marker.points.append(start_point)
            line_marker.points.append(end_point)
            marker_array.markers.append(line_marker)

            # --- Text marker (matching color) ---
            mid_x = (start_world[0] + end_world[0]) / 2.0
            mid_y = (start_world[1] + end_world[1]) / 2.0

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "map_lines"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = mid_x
            text_marker.pose.position.y = mid_y
            text_marker.pose.position.z = 0.5
            text_marker.pose.orientation.w = 1.0
            text_marker.text = line['name']
            text_marker.scale.z = 0.5
            text_marker.color.r = r
            text_marker.color.g = g
            text_marker.color.b = b
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)

        self.line_pub.publish(marker_array)

    def line_intersection(self, line1, line2):
        """Check if two line segments intersect"""
        x1, y1 = line1[0]
        x2, y2 = line1[1]
        x3, y3 = line2[0]
        x4, y4 = line2[1]

        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:  # lines are parallel
            return False

        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

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
            (_, _, yaw) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

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

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        marker.scale.x = self.marker_params['length']
        marker.scale.y = self.marker_params['width']
        marker.scale.z = self.marker_params['height']

        # Set color based on intersection
        marker.color.r = 1.0  # Red default
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        if intersecting:
            marker.color.g = 1.0  # Yellow marker for intersections
            if self.stop_detector.just_stopped():
                rospy.loginfo_throttle(1.0, f"Facing {intersecting_line_name}!")
                self.tts_node.announce_status(f"Facing {intersecting_line_name}!")
        else:
            if self.stop_detector.just_stopped():
                rospy.loginfo_throttle(2.0, "No intersection detected")
                self.tts_node.announce_status("Not close to anything!")

        self.arrow_pub.publish(marker)


if __name__ == '__main__':
    try:
        RobotDirectionMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
