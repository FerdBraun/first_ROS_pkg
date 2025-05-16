1. Install ROS2 (foxy for 20.04) or other ([Video guide](https://www.youtube.com/watch?v=uWzOk0nkTcI))
2. Create a folder "ros_dev" for excample
3. make an ```src``` dir in it
5. Build an empty workspace by running ```colcon build --symlink-install``` while in ```ros_dev``` dir
6. Open ```src``` dir and clone this repo
7. Go to the ```ros_dev``` dir and rebuild the workspace by running the same command
8. source the dir by running ```source install/setup.bash```


dependencies:
1.
```
sudo apt-get install freeglut3-dev
sudo apt install libusb-dev
sudo apt-get update
sudo apt-get install libusb-1.0-0-dev
```
2.
install libfreekeenect 

sudo make install
https://github.com/OpenKinect/libfreenect

3.
```
sudo apt-get install ros-foxy-depth-image-proc
sudo apt-get install ros-foxy-camera-info-manager
```
4.
```
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-slam-toolbox
sudo apt install ros-foxy-controller-manager
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controllers
sudo apt install ros-foxy-gazebo-ros2-control
sudo apt install ros-foxy-tf-transformations
```
5.
```
sudo pip3 install transforms3d
pip install pynput
```
6.
```
sudo apt install ros-foxy-twist-mux
sudo apt install ros-foxy-nav2-*
sudo apt install ros-foxy-rtabmap-*
sudo apt install ros-foxy-imu-tools
```
7.
```
sudo apt install ros-foxy-octomap-server ros-foxy-octomap-msgs ros-foxy-octomap-rviz-plugins
```


RUN - Sumulation
Main file 
```
ros2 launch my_gazebo gazebo_test.launch.py world:=~/Desktop/ros_dev/src/my_gazebo/world/test_world.world
```
SLAM + Odometry
```
ros2 launch my_gazebo rmap.launch.py 

```
Costmap
```
ros2 launch my_gazebo nav2_bringup.launch.py 
```
Moving Nodes 
```
ros2 launch my_gazebo movement.launch.py
```
RVIZ
```
rviz2 -d ~/Desktop/ros_dev/src/my_gazebo/rviz_config/rviz_view.rviz
```
FOR AUTONOMOUS EXPLORATION 
```
ros2 launch my_gazebo explorer.launch.py 
```

RUN - Real robot

Main file 
```
ros2 launch real realLife.launch.py 
```
Kinect
```
ros2 launch kinect_ros2 pointcloud.launch.py
```

Run and connect  Imu server with android device, then

```
ros2 launch real IMUBridge.launch.py 
```
Dont forget to change IPs


SLAM + Odometry
```
ros2 launch real rmap.launch.py 

```
Costmap
```
ros2 launch real nav2_bringup.launch.py 
```
Moving Nodes 
```
ros2 launch real movement.launch.py
```
Dont forget to run robot movement server

RVIZ
```
rviz2 -d ~/Desktop/ros_dev/src/my_gazebo/rviz_config/rviz_view.rviz
```
FOR AUTONOMOUS EXPLORATION 
```
ros2 launch real explorer.launch.py 
```










sourses:

https://github.com/fadlio/kinect_ros2?tab=readme-ov-file (modified)
https://github.com/dawan0111/Simple-2D-LiDAR-Odometry (modified)
https://github.com/107-systems/l3xz_sweep_scanner (modified)

