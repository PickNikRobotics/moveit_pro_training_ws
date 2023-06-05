# apriltag_ros_python

This is a simple ROS 2 package that wraps the [`dt_apriltags`](https://github.com/duckietown/lib-dt-apriltags) Python package for AprilTag detection.

To set up, you must first install `dt_apriltags` with:

```
pip3 install dt_apriltags
```

Then, you can run the nodes directly:

Server node:

```
# Default args
ros2 launch apriltag_ros_python apriltag_detection_server.launch.py

# Other args
ros2 launch apriltag_ros_python apriltag_detection_server.launch.py visualize:=True apriltag_family:=tag36h11 apriltag_size:=0.2
```

Client node:

```
# Test mode
ros2 launch apriltag_ros_python apriltag_detection_client.launch.py test_mode:=true

# Run with real topics
ros2 launch apriltag_ros_python apriltag_detection_client.launch.py test_mode:=false camera_info_topic:=/wrist_mounted_camera/color/camera_info image_topic:=/wrist_mounted_camera/color/image_raw
```
