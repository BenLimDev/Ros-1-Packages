# ROS1 Packages

Collection of ROS1 packages for AIROST.

## Prerequisites
- ROS1 (Melodic/Noetic)
- Catkin workspace

## Usage
```
# Clone the repository
cd
git clone https://github.com/BenLimDev/Ros-1-Packages.git

# Copy the package you need to your catkin workspace
cp -r Ros-1-Packages/package_name ~/catkin_ws/src/

# Build
cd ~/catkin_ws
catkin_make

# Source
source devel/setup.bash
```

## Packages

- **nav**: Navigation package

## Run

```bash
roslaunch nav navigation.launch
```
# check_moving.py
## Provides two detection methods:
    - check_stopped(): Returns current stopped state, continously
    - just_stopped(): Returns True only on transition from moving to stopped, only once
### As a Library in Your Code:
```#!/usr/bin/env python
import rospy
from your_package.robot_stop_detector import RobotStopDetector

class MyRobotController:
    def __init__(self):
        rospy.init_node('my_robot_controller')
        
        # Initialize the stop detector
        self.stop_detector = RobotStopDetector()
        
        # You can modify the thresholds like this:
        self.stop_detector.position_threshold = 0.02  # meters (default: 0.01)
        self.stop_detector.orientation_threshold = 0.1  # radians (default: 0.05)
        self.stop_detector.time_threshold = rospy.Duration(3)  # seconds (default: 2)
        self.rate = rospy.Rate(10)  # 10Hz
        
    def run(self):
        while not rospy.is_shutdown():
            # Example 1: Check current stopped state
            if self.stop_detector.check_stopped():
                rospy.loginfo("Robot is currently stopped")
            
            # Example 2: Detect the moment when robot stops
            if self.stop_detector.just_stopped():
                rospy.loginfo("Robot just stopped moving!")
            
            self.rate.sleep()
```



# For adding other packages:
- Copy another package from your catkin_ws
```
cd Ros-1-Packages
git pull
cp -r ~/catkin_ws/src/another_package ./another_package
```
- Pull, Add, commit, and push
```
git add another_package/
git commit -m "Add another_package"
git push
```
