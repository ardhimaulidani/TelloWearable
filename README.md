# DJI Tello - ROS2 
Using wearable device to control DJI Tello with ROS2 Platform

## Reference 
- Wearable Device : 
https://github.com/Soeromiber/SAFMC_Wearable
- DJI Tello ROS2 : 
https://github.com/clydemcqueen/tello_ros

## Installation
### 1. Setup Linux and ROS Environment
Install ROS2 Foxy with the ```ros-foxy-desktop``` option. Also install these additional package
```bash
sudo apt install libasio-dev
sudo apt install ros-foxy-cv-bridge ros-foxy-camera-calibration-parsers
```
### 2. Install ```tello_ros```
```bash
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
source /opt/ros/foxy/setup.bash
# If you didn't intall Gazebo, skip tello_gazebo while building:
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo
```
### 3. Install ```TelloWearable```
```bash
cd ~/tello_ros_ws/src
git clone https://github.com/ardhimaulidani/TelloWearable.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo
```
## Usage
1. Run ```teleop``` node
```bash
cd ~/tello_ros_ws
source install/setup.bash
ros2 launch tello_driver teleop_launch.py
```
2. Run ```tello_wearable``` node
```bash
ros2 launch TelloWearable tello_wearable
```
3. Run MicroRos-Agent
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
