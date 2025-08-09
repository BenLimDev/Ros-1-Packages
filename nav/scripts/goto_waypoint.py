#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('navigate_waypoints')

        # Connect to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        # Waypoint definitions (edit these as needed)
        self.waypoints = {
            "origin": Pose(
                position=Point(0.0, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
            "living": Pose(
                position=Point(4.32, 3.76, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
            "door": Pose(
                position=Point(5.31, 3.56, 0.0),
                orientation=Quaternion(1.571, 0.0, 0.0, 1.0)
            ),
        }

        # Subscribe to topic to trigger navigation
        self.sub = rospy.Subscriber('/go_to_waypoint', String, self.handle_waypoint_request)

        rospy.loginfo("Ready to receive waypoint commands on /go_to_waypoint")

    def handle_waypoint_request(self, msg):
        waypoint_name = msg.data.strip().lower()
        self.go_to_waypoint(waypoint_name)

    def go_to_waypoint(self, name, timeout=150.0):
        if name not in self.waypoints:
            rospy.logwarn(f"[NAV] Waypoint '{name}' not found.")
            return False

        pose = self.waypoints[name]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        rospy.loginfo(f"[NAV] Navigating to: {name}")
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(timeout))

        if finished and self.client.get_result():
            rospy.loginfo(f"[NAV] Reached waypoint: {name}")
            return True
        else:
            rospy.logwarn(f"[NAV] Timeout or failure at waypoint: {name}")
            self.client.cancel_goal()
            return False

if __name__ == '__main__':
    try:
        nav = WaypointNavigator()
        rospy.spin()  # Wait for incoming waypoint messages
    except rospy.ROSInterruptException:
        pass
