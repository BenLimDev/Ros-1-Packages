#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Twist
import threading
import queue
from collections import deque
import math
import pyzed.sl as sl

# ===== FIRST CODE START ======
class BagDetector:
    def __init__(self):
        self.distance_pub = rospy.Publisher('/bag_distance', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher('/bag_angle', Float32, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        rospy.Subscriber('/pointing_hand', Int32, self.pointing_hand_callback)
        self.bag_side = 0
        rospy.Subscriber('/reach_distance', Bool, self.reach_distance_callback)

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        self.runtime_parameters = sl.RuntimeParameters()

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr(f"Error {err} opening ZED camera. Exiting.")
            exit(1)

        self.image_zed = sl.Mat()
        self.depth_zed = sl.Mat()
        self.point_cloud = sl.Mat()
        self.frame_queue = queue.Queue(maxsize=10)

        self.model = YOLO("yolo12n.pt")
        self.model.cuda()

        self.processing_thread = threading.Thread(target=self.capture)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def pointing_hand_callback(self, msg):
        if msg:
            self.bag_side = msg.data

    def reach_distance_callback(self, msg):
        if msg.data:
            rospy.loginfo("Target distance reached. Stopping robot movement.")
            stop_cmd = Twist()
            self.velocity_pub.publish(stop_cmd)

    def process_frames(self):
        while not rospy.is_shutdown():
            try:
                if not self.frame_queue.empty():
                    depth, image = self.frame_queue.get()
                    self.color_image = image.copy()
                    width = self.color_image.shape[1]
                    middle_x = width / 2

                    results = self.model(image, stream=False)
                    people_info = []

                    for boxes in results[0].boxes:
                        cls_id = int(boxes.cls[0])
                        confidence = boxes.conf[0]
                        if cls_id == 0 and confidence > 0.60:
                            box = boxes.xyxy[0].tolist()
                            x1, y1, x2, y2 = map(int, box)
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2
                            height = image.shape[0]
                            err, depth_value = depth.get_value(center_x, height- center_y)
                            if err == sl.ERROR_CODE.SUCCESS and not math.isnan(depth_value) and depth_value > 0:
                                people_info.append((depth_value, center_x, center_y, box))

                    if people_info:
                        people_info.sort(key=lambda x: x[0])  # Closest first
                        closest_depth, closest_x, closest_y, box = people_info[0]

                        # Draw all boxes, highlight the closest
                        for i, (d, cx, cy, b) in enumerate(people_info):
                            x1, y1, x2, y2 = map(int, b)
                            color = (0, 0, 255) if i == 0 else (0, 255, 0)
                            label = "Closest" if i == 0 else "Person"
                            cv2.rectangle(self.color_image, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(self.color_image, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                        offset = closest_x - middle_x
                        angle = math.atan2(closest_y, offset)
                        angle = angle if offset < 0 else -angle
                        self.distance_pub.publish(closest_depth)
                        self.angle_pub.publish(angle)
                        print(f"\nClosest Person Distance = {closest_depth:.2f} mm, Angle = {angle:.4f} rad\n")
                    self.color_image = cv2.flip(self.color_image, 0)
                    cv2.imshow('Person Detection', self.color_image)
                    cv2.waitKey(1)

            except Exception as e:
                rospy.logerr(f"Error in processing thread: {str(e)}")

    def capture(self):
        while not rospy.is_shutdown():
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                self.color_image = self.image_zed.get_data()
                self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGRA2BGR)
                self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)
                if not self.frame_queue.full():
                    self.frame_queue.put((self.depth_zed, self.color_image))
            rospy.Rate(30).sleep()

# ===== SECOND CODE START ======
class TurtleBotApproach:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)
        rospy.Subscriber('/bag_distance', Float32, self.distance_callback)
        rospy.Subscriber('/bag_angle', Float32, self.angle_callback)

        self.target_distance = 0.3  # meters
        self.target_angle = 1.5     # radians

        self.max_linear_vel = 0.4
        self.max_angular_vel = 0.6
        self.cmd_vel = Twist()
        self.distance_to_bag = None
        self.angle_to_bag = None
        self.has_reached_distance = False

    def distance_callback(self, msg):
        self.distance_to_bag = msg.data / 1000  # Convert mm to meters
        self.update_velocity()

    def angle_callback(self, msg):
        self.angle_to_bag = msg.data
        self.update_velocity()

    def update_velocity(self):
        if self.distance_to_bag is None or self.angle_to_bag is None:
            rospy.loginfo("Waiting for distance and angle measurements...")
            return

        error_dist = self.distance_to_bag - self.target_distance
        error_ang = self.angle_to_bag - self.target_angle

        # Stop if within range
        if abs(error_dist) <= 0.3:
            if not self.has_reached_distance:
                rospy.loginfo("Target distance reached. Stopping robot.")
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                self.vel_pub.publish(self.cmd_vel)
                self.reach_distance_pub.publish(True)
                self.has_reached_distance = True
            return
        else:
            if self.has_reached_distance:
                rospy.loginfo("Person moved away. Resuming tracking.")
                self.has_reached_distance = False
                self.reach_distance_pub.publish(False)

        # ==== New: Variable velocity based on distance ====
        if self.distance_to_bag > 1.5:
            self.cmd_vel.linear.x = 0.4
        elif self.distance_to_bag > 1.0:
            self.cmd_vel.linear.x = 0.3
        elif self.distance_to_bag > 0.6:
            self.cmd_vel.linear.x = 0.2
        else:
            self.cmd_vel.linear.x = 0.05
        # ===============================================

        if error_ang > 0.5 and self.angle_to_bag != 0:
            self.cmd_vel.angular.z = -self.max_angular_vel     # turn right
        elif error_ang < -0.5 and self.angle_to_bag != 0:
            self.cmd_vel.angular.z = self.max_angular_vel      # turn left
        else:
            self.cmd_vel.angular.z = 0

        rospy.loginfo(f'VELOCITY - Linear: {self.cmd_vel.linear.x:.2f}, Angular: {self.cmd_vel.angular.z:.2f}')
        self.vel_pub.publish(self.cmd_vel)

    def run(self):
        rospy.spin()

# ===== MAIN ENTRY POINT =======
if __name__ == '__main__':
    try:
        rospy.init_node('person_tracking_node')
        detector = BagDetector()
        turtlebot_approach = TurtleBotApproach()
        detector_thread = threading.Thread(target=detector.process_frames)
        detector_thread.start()
        turtlebot_approach.run()
    except rospy.ROSInterruptException:
        pass
