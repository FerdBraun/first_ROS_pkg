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
```
sudo apt-get install ros-foxy-depth-image-proc
sudo apt-get install ros-foxy-camera-info-manager
```
3.
```
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-controller-manager
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controllers
sudo apt install ros-foxy-gazebo-ros2-control
sudo apt install ros-foxy-tf-transformations
```
4.
```
sudo pip3 install transforms3d
pip install pynput
```
5.
```
sudo apt install ros-foxy-twist-mux
sudo apt install ros-foxy-nav2-*
sudo apt install ros-foxy-rtabmap-*
sudo apt install ros-foxy-imu-tools
```
6.
```
sudo apt install ros-foxy-octomap-server 
sudo apt install ros-foxy-octomap-msgs 
sudo apt install ros-foxy-octomap-rviz-plugins
```

RUN
Main file 
```
ros2 launch my_gazebo gazebo_test.launch.py
```
Odometry + SLAM
```
ros2 launch my_gazebo rmap.launch.py
```
Path and navigations (via publishing point)
```
ros2 launch my_gazebo nav2_bringup.launch.py
```
Moving Nodes 
```
ros2 launch my_gazebo movement.launch.py
```
RVIZ
```
ros2 launch my_gazebo rviz.launch.py
```
FOR AUTONOMOUS EXPLORATION 
```
ros2 launch my_gazebo explorer.launch.py 
```





sourses:

https://github.com/fadlio/kinect_ros2?tab=readme-ov-file


