#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

class DirectionMarker:
    def __init__(self):
        rospy.init_node('direction_marker_node')
        self.marker_pub = rospy.Publisher('/direction_marker', Marker, queue_size=10, latch=True)
        rospy.loginfo("Publisher created!")
        
        # Test marker (publish once at startup)
        test_marker = Marker()
        test_marker.header.frame_id = "odom"
        test_marker.type = Marker.ARROW
        test_marker.action = Marker.ADD
        test_marker.pose.position.x = 0
        test_marker.pose.position.y = 0
        test_marker.pose.orientation.w = 1.0  # Neutral orientation
        test_marker.scale.x = 0.3
        test_marker.scale.y = 0.05
        test_marker.scale.z = 0.05
        test_marker.color.a = 1.0
        test_marker.color.r = 1.0
        self.marker_pub.publish(test_marker)
        
        rospy.loginfo("Published test marker!")
    
if __name__ == '__main__':
    try:
        marker = DirectionMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass