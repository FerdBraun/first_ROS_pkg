1. Install ROS2 (foxy for 20.04) or other ([Video guide](https://www.youtube.com/watch?v=uWzOk0nkTcI))
2. Create a folder "ros_dev" for excample
3. make an ```src``` dir in it
5. Build an empty workspace by running ```colcon build --symlink-install``` while in ```ros_dev``` dir
6. Open ```src``` dir and clone this repo
7. Go to the ```ros_dev``` dir and rebuild the workspace by running the same command
8. source the dir by running ```source install/setup.bash```
9. run ```ros2 launch my_gazebo gazebo_test.launch.py world:=~/Desktop/ros_dev/src/my_gazebo/world/test_world.world```. Dont forget to change the path to your ```ros_dev```

to run mapping part run ```ros2 launch lidar_odometry lidar_odom.launch.py```


for real devices run 
```ros2 launch l3xz_sweep_scanner laser.launch.py``` for lidar
and
```ros2 launch kinect_ros2 pointcloud.launch.py``` for kinect

sourses:

https://github.com/fadlio/kinect_ros2?tab=readme-ov-file
https://github.com/dawan0111/Simple-2D-LiDAR-Odometry (modified)
https://github.com/107-systems/l3xz_sweep_scanner (modified)

dependencies:
todo ~ ~
pynput,
