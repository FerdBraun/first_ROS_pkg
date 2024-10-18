set(_AMENT_PACKAGE_NAME "kinect_ros2")
set(kinect_ros2_VERSION "0.0.1")
set(kinect_ros2_MAINTAINER "Fernando de Lio <fernandodelio@gmail.com>")
set(kinect_ros2_BUILD_DEPENDS "libfreenect" "rclcpp" "sensor_msgs" "cv_bridge" "image_transport" "camera_info_manager" "depth_image_proc")
set(kinect_ros2_BUILDTOOL_DEPENDS "ament_cmake")
set(kinect_ros2_BUILD_EXPORT_DEPENDS "libfreenect" "rclcpp" "sensor_msgs" "cv_bridge" "image_transport" "camera_info_manager" "depth_image_proc")
set(kinect_ros2_BUILDTOOL_EXPORT_DEPENDS )
set(kinect_ros2_EXEC_DEPENDS "image_tools" "libfreenect" "rclcpp" "sensor_msgs" "cv_bridge" "image_transport" "camera_info_manager" "depth_image_proc")
set(kinect_ros2_TEST_DEPENDS )
set(kinect_ros2_GROUP_DEPENDS )
set(kinect_ros2_MEMBER_OF_GROUPS )
set(kinect_ros2_DEPRECATED "")
set(kinect_ros2_EXPORT_TAGS)
list(APPEND kinect_ros2_EXPORT_TAGS "<build_type>ament_cmake</build_type>")