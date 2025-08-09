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

## Run

```bash
roslaunch nav navigation.launch
```
# How to navigate to waypoint (goto_waypoint.py)
- Must run with navigation.launch
- Using with python:
```
from path_to/navigate_waypoints import WaypointNavigator  

if __name__ == "__main__":
    rospy.init_node("controller")# How to use spin and stop/allign at the first person spin.py

Using with python:

```from std_msgs.msg import String


pub = rospy.Publisher("/rotator/control", String, queue_size=1)

rospy.sleep(0.2)

pub.publish("start")     # or "continue" / "stop"

print("sent: start")

rospy.sleep(0.5)```


Use rostopic publish:

```rostopic pub /rotator/control std_msgs/String "start" #or "continue" / "stop"``` 
    nav = WaypointNavigator()          
    nav.go_to_waypoint("living")      
    rospy.loginfo("Done")
```

- Use rostopic publish:
```
rostopic pub /go_to_waypoint std_msgs/String "data: 'living'"
```

# How to use spin and stop/allign at the first person spin.py
- Continue will skip the current person by ignoring any person from the current angle and look for the next person
- Using with python:
```
from std_msgs.msg import String

pub = rospy.Publisher("/rotator/control", String, queue_size=1)
rospy.sleep(0.2)
pub.publish("start")     # or "continue" / "stop"
print("sent: start")
rospy.sleep(0.5)
```

- Use rostopic publish:
```
rostopic pub /rotator/control std_msgs/String "start" #or "continue" / "stop"
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
