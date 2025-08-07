#!/home/ben-focal-fossa/venvs/yolo12/bin/python3

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
from turtlebot_approach import TurtleBotApproach

# ===== FIRST CODE START ======
class PersonDetector:
    def __init__(self):
        self.distance_pub = rospy.Publisher('/person_distance', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher('/person_angle', Float32, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        rospy.Subscriber('/pointing_hand', Int32, self.pointing_hand_callback)
        self.person_side = 0
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
            self.person_side = msg.data

    def reach_distance_callback(self, msg):
        if msg.data:
            #rospy.loginfo("Target distance reached. Stopping robot movement.")
            stop_cmd = Twist()
            self.velocity_pub.publish(stop_cmd)
            
    def process_frames(self):
        while not rospy.is_shutdown():
            if self.frame_queue.empty():
                continue

            depth, image = self.frame_queue.get()
            self.color_image = image.copy()
            h, w = self.color_image.shape[:2]
            middle_x = w // 2

            results = self.model(image)
            people_info = []

            # 1) First pass: collect all valid detections
            for det in results[0].boxes:
                cls_id     = int(det.cls[0])
                confidence = float(det.conf[0])
                if cls_id != 0 or confidence < 0.60:
                    continue  # skip non-person or low-conf

                # Extract box coords
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # Bounds check & fetch depth from numpy
                if 0 <= cy < depth.shape[0] and 0 <= cx < depth.shape[1]:
                    d = float(depth[cy, cx])
                    if not math.isnan(d) and d > 0:
                        people_info.append((d, cx, cy, (x1, y1, x2, y2)))

            # 2) If any people found, pick the closest and draw
            if people_info:
                people_info.sort(key=lambda x: x[0])  # nearest first
                closest = people_info[0]

                for idx, (d, cx, cy, (x1, y1, x2, y2)) in enumerate(people_info):
                    color = (0,0,255) if idx==0 else (0,255,0)
                    label = "Closest" if idx==0 else "Person"
                    cv2.rectangle(self.color_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(self.color_image, label, (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # publish only closest
                d, cx, cy, _ = closest
                offset = cx - middle_x
                angle  = math.atan2(cy, offset)
                angle  = -angle if offset > 0 else angle

                self.distance_pub.publish(d)
                self.angle_pub.publish(angle)

            # 3) Show image
            cv2.imshow('Person Detection', cv2.flip(self.color_image, 0))
            cv2.waitKey(1)

    def capture(self):
        while not rospy.is_shutdown():
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_zed,   sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)

                # Immediately pull a CPU numpy array of depth:
                depth_np = self.depth_zed.get_data()  # shape H×W float32

                # convert and queue
                img_np = cv2.cvtColor(self.image_zed.get_data(), cv2.COLOR_BGRA2BGR)
                if not self.frame_queue.full():
                    self.frame_queue.put((depth_np, img_np))
            rospy.Rate(30).sleep()


# # ===== SECOND CODE START ======
# class TurtleBotApproach:
#     def __init__(self):
#         self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
#         self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)
#         rospy.Subscriber('/person_distance', Float32, self.distance_callback)
#         rospy.Subscriber('/person_angle', Float32, self.angle_callback)

#         self.target_distance = 0.5  # meters and self.angle_to_person != 0
#         self.target_angle = 1.571     # radians

#         self.cmd_vel = Twist()
#         self.distance_to_person = None
#         self.angle_to_person = None
#         self.has_reached_distance = False

#     def distance_callback(self, msg):
#         self.distance_to_person = msg.data / 1000  # Convert mm to meters
#         self.update_velocity()

#     def angle_callback(self, msg):
#         self.angle_to_person = msg.data
#         self.update_velocity()

#     def update_velocity(self):
#         if self.distance_to_person is None or self.angle_to_person is None:
#             #rospy.loginfo("Waiting forospy.Rate(30).sleep()rospy.Rate(30).sleep()r distance and angle measurements...")
#             return

#         error_dist = self.distance_to_person - self.target_distance
#         error_ang = self.angle_to_person - self.target_angle


#         # ===== MODIFIED: Stop linear.x when in range, BUT allow angular.z turning =====
#         if abs(error_dist) <= 0.1:
#             self.cmd_vel.linear.x = 0
#             if error_ang > 1.2 and self.angle_to_person != 0:
#                 self.cmd_vel.angular.z = -1.5     # turn right
#                 print(f"\nangle = {error_ang}turning Right most")

#             elif error_ang > 0.9 and self.angle_to_person != 0:
#                 self.cmd_vel.angular.z = -1.0     # turn right more     
#                 print(f"\nangle = {error_ang}turning Right more")

#             elif error_ang > 0.2 and self.angle_to_person != 0:
#                 self.cmd_vel.angular.z = -0.7     # turn right most   
#                 print(f"\nangle = {error_ang}turning Right")

#             elif error_ang < -3.4 and self.angle_to_person != 0:
#                 self.cmd_vel.angular.z = 1.5      # turn left
#                 print(f"\nangle = {error_ang}turning Left most")
             
#             elif error_ang < -2.9 and self.angle_to_person != 0:
#                 self.cmd_vel.angular.z = 1.0     # turn left more
#                 print(f"\nangle = {error_ang}turning Left more")
              
#             elif error_ang < -2.5 and self.angle_to_person != 0:
#                 self.cmd_vel.angular.z = 0.7      # turn left most      
#                 print(f"\nangle = {error_ang}turning Left")
              
#             else:
#                 self.cmd_vel.angular.z = 0
#                 print(f"\nangle = {error_ang} not rospy.Rate(30).sleep(30) turning")
               

#             if not self.has_reached_distance:
#                 #rospy.loginfo("Target distance reached. Linear motion stopped. Angular turning active.")
#                 self.vel_pub.publish(self.cmd_vel)
#                 self.reach_distance_pub.publish(True)
#                 self.has_reached_distance = True
#             else:
#                 self.vel_pub.publish(self.cmd_vel)
#             return
#         else:
#             if self.has_reached_distance:
#                 #rospy.loginfo("Person moved away. Resuming linear tracking.")
#                 self.has_reached_distance = False
#                 self.reach_distance_pub.publish(False)

#         # ==== Variable linear velocity ====
#         if self.distance_to_person > self.target_distance+0.9:
#             self.cmd_vel.linear.x = 1.0
#         elif self.distance_to_person > self.target_distance+0.8:
#             self.cmd_vel.linear.x = 0.8
#         elif self.distance_to_person > self.target_distance+0.7:
#             self.cmd_vel.linear.x = 0.6
#         elif self.distance_to_person > self.target_distance+0.5:
#             self.cmd_vel.linear.x = 0.3
#         elif self.distance_to_person > self.target_distance+0.3:
#             self.cmd_vel.linear.x = 0.2
#         elif self.distance_to_person > self.target_distance+0.1:
#             self.cmd_vel.linear.x = 0.1
#         else:
#             self.cmd_vel.linear.x = 0

#         # ==== Angular velocity ====
#         if error_ang > 0.9 and self.angle_to_person != 0:
#             self.cmd_vel.angular.z = -1.4     # turn right
#             print(f"\nangle = {error_ang}turning Right most")
#         elif error_ang > 0.6 and self.angle_to_person != 0:
#             self.cmd_vel.angular.z = -1.2     # turn right more     
#             print(f"\nangle = {error_ang}turning Right more")
#         elif error_ang > 0.2 and self.angle_to_person != 0:
#             self.cmd_vel.angular.z = -0.7     # turn right most   
#             print(f"\nangle = {error_ang}turning Right")

#         elif error_ang < -2.7 and self.angle_to_person != 0:
#             self.cmd_vel.angular.z = 1.4     # turn left
#             print(f"\nangle = {error_ang}turning Left most")
            
#         elif error_ang < -2.48 and self.angle_to_person != 0:
#             self.cmd_vel.angular.z = 1.2     # turn left more
#             print(f"\nangle = {error_ang}turning Left more")
            
#         elif error_ang < -2.3 and self.angle_to_person != 0:
#             self.cmd_vel.angular.z = 0.7      # turn left most      
#             print(f"\nangle = {error_ang}turning Left")
#         else:
#             self.cmd_vel.angular.z = 0
#             print("\nstop turning\n")
#             print(f"\nangle = {error_ang},stop turning")

#         #rospy.loginfo(f'VELOCITY - Linear: {self.cmd_vel.linear.x:.2f}, Angular: {self.cmd_vel.angular.z:.2f}')
#         self.vel_pub.publish(self.cmd_vel)

#     def run(self):
#         rospy.spin()

# ===== MAIN ENTRY POINT =======
if __name__ == '__main__':
    try:
        rospy.init_node('person_tracking_node')
        detector = PersonDetector()
        turtlebot_approach = TurtleBotApproach()
        detector_thread = threading.Thread(target=detector.process_frames)
        detector_thread.start()
        turtlebot_approach.run()
    except rospy.ROSInterruptException:
        pass
