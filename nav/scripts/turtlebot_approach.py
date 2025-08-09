#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class TurtleBotApproach:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reach_distance_pub = rospy.Publisher('/reach_distance', Bool, queue_size=10)
        rospy.Subscriber('/person_distance', Float32, self.distance_callback)
        rospy.Subscriber('/person_angle', Float32, self.angle_callback)

        self.target_distance = 0.7  # meters
        self.target_angle = 0   # radians

        self.cmd_vel = Twist()
        self.distance_to_person = None
        self.angle_to_person = None
        self.has_reached_distance = False

    def distance_callback(self, msg):
        self.distance_to_person = msg.data / 1000.0  # mm→m
        self.update_velocity()

    def angle_callback(self, msg):
        self.angle_to_person = msg.data
        self.update_velocity()

    def update_velocity(self):
        if self.distance_to_person is None or self.angle_to_person is None:
            #rospy.loginfo("Waiting forospy.Rate(30).sleep()rospy.Rate(30).sleep()r distance and angle measurements...")
            return

        error_dist = self.distance_to_person - self.target_distance
        error_ang = self.angle_to_person - self.target_angle


        # ===== MODIFIED: Stop linear.x when in range, BUT allow angular.z turning =====
        if abs(error_dist) <= 0.1:
            self.cmd_vel.linear.x = 0
            if error_ang > 0.7 and self.angle_to_person != 0:
                self.cmd_vel.angular.z = -1.7     # turn right
                print(f"\nangle = {error_ang}turning Right most")

            elif error_ang > 0.5 and self.angle_to_person != 0:
                self.cmd_vel.angular.z = -1.2     # turn right more    
                print(f"\nangle = {error_ang}turning Right more")

            elif error_ang > 0.2 and self.angle_to_person != 0:
                self.cmd_vel.angular.z = -0.7     # turn right most   
                print(f"\nangle = {error_ang}turning Right")

            elif error_ang < -0.7 and self.angle_to_person != 0:
                self.cmd_vel.angular.z = 1.7      # turn left
                print(f"\nangle = {error_ang}turning Left most")
             
            elif error_ang < -0.5 and self.angle_to_person != 0:
                self.cmd_vel.angular.z = 1.2     # turn left more
                print(f"\nangle = {error_ang}turning Left more")
              
            elif error_ang < -0.2 and self.angle_to_person != 0:
                self.cmd_vel.angular.z = 0.7      # turn left most      
                print(f"\nangle = {error_ang}turning Left")
              
            else:
                self.cmd_vel.angular.z = 0
                print(f"\nangle = {error_ang} not turning")
               

            if not self.has_reached_distance:
                #rospy.loginfo("Target distance reached. Linear motion stopped. Angular turning active.")
                self.vel_pub.publish(self.cmd_vel)
                self.reach_distance_pub.publish(True)
                self.has_reached_distance = True
            else:
                self.vel_pub.publish(self.cmd_vel)
            return
        else:
            if self.has_reached_distance:
                #rospy.loginfo("Person moved away. Resuming linear tracking.")
                self.has_reached_distance = False
                self.reach_distance_pub.publish(False)

        # ==== Variable linear velocity ====
        if self.distance_to_person > self.target_distance+1.5:
            self.cmd_vel.linear.x = 2.0
        elif self.distance_to_person > self.target_distance+1.3:
            self.cmd_vel.linear.x = 1.5
        elif self.distance_to_person > self.target_distance+1.1:
            self.cmd_vel.linear.x = 1.0
        elif self.distance_to_person > self.target_distance+0.9:
            self.cmd_vel.linear.x = 0.8
        elif self.distance_to_person > self.target_distance+0.7:
            self.cmd_vel.linear.x = 0.6
        elif self.distance_to_person > self.target_distance+0.5:
            self.cmd_vel.linear.x = 0.3
        elif self.distance_to_person > self.target_distance+0.3:
            self.cmd_vel.linear.x = 0.2
        elif self.distance_to_person > self.target_distance+0.1:
            self.cmd_vel.linear.x = 0.1
        else:
            self.cmd_vel.linear.x = 0

        # ==== Angular velocity ====
        if error_ang > 0.8 and self.angle_to_person != 0:
            self.cmd_vel.angular.z = -1.7     # turn right
            print(f"\nangle = {error_ang}turning Right most")

        elif error_ang > 0.6 and self.angle_to_person != 0:
            self.cmd_vel.angular.z = -1.2     # turn right more    
            print(f"\nangle = {error_ang}turning Right more")

        elif error_ang > 0.2 and self.angle_to_person != 0:
            self.cmd_vel.angular.z = -0.7     # turn right most   
            print(f"\nangle = {error_ang}turning Right")

        elif error_ang < -0.8 and self.angle_to_person != 0:
            self.cmd_vel.angular.z = 1.7      # turn left
            print(f"\nangle = {error_ang}turning Left most")
            
        elif error_ang < -0.6 and self.angle_to_person != 0:
            self.cmd_vel.angular.z = 1.2     # turn left more
            print(f"\nangle = {error_ang}turning Left more")
            
        elif error_ang < -0.2 and self.angle_to_person != 0:
            self.cmd_vel.angular.z = 0.7      # turn left most      
            print(f"\nangle = {error_ang}turning Left")
            
        else:
            self.cmd_vel.angular.z = 0
            print(f"\nangle = {error_ang} not turning")

        #rospy.loginfo(f'VELOCITY - Linear: {self.cmd_vel.linear.x:.2f}, Angular: {self.cmd_vel.angular.z:.2f}')
        self.vel_pub.publish(self.cmd_vel)

    def run(self):
        rospy.spin()
