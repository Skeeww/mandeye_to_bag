# Tools to Convert Mandeye Data to ROSBag

This repository contains a simple tool that enables the conversion of Mandeye data to ROSBag.

[![ROS 2 Build](https://github.com/michalpelka/mandeye_to_bag/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/michalpelka/mandeye_to_bag/actions/workflows/ros2-build.yml)

## ROS 2

### Building

To build the tool, follow these steps:

1. Create a ROS1 workspace:
```
mkdir -p ~/mandeye_ws/src
```

2. Clone the repository:
```
cd ~/mandeye_ws/src
git clone --recurse-submodules https://github.com/Skeeww/mandeye_to_bag.git
```

3. Install dependencies
```
cd ~/mandeye_ws/src/mandeye_to_rosbag2
rosdep update && rosdep install --from-paths . --ignore-src -r -y
```

4. Build the workspace:
```
cd ~/mandeye_ws
colcon build
```

### Usage

To convert Mandeye data, run the following command (don't forget to source the workspace):
```
ros2 run mandeye_to_rosbag2 mandeye_to_rosbag <input scan_folder> <output bag_folder> --lines [number of lines] --type [pointcloud2 | livox] --points [number of points]
```
