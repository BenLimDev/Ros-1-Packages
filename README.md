# ROS1 Packages

Collection of ROS1 packages for robotics projects.

## Prerequisites
- ROS1 (Melodic/Noetic)
- Catkin workspace

## Usage
```
# Clone the repository
git clone https://github.com/BenLimDev/Ros-1-Packages.git

# Copy the package you need to your catkin workspace
cd ~/catkin_ws/src
cp -r /path/to/Ros-1-Packages/nav ./

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

# For adding other packages:
- Copy another package from your catkin_ws

cp -r ~/catkin_ws/src/another_package ./another_package

- Add, commit, and push

git add another_package/
git commit -m "Add another_package"
git push
