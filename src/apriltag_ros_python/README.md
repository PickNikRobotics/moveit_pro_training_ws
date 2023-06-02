# apriltag_ros_python

This is a simple ROS 2 package that wraps the [`dt_apriltags`](https://github.com/duckietown/lib-dt-apriltags) Python package for AprilTag detection.

To set up, you must first install `dt_apriltags` with:

```
pip3 install dt_apriltags
```

Then, you can run the nodes directly:

```
ros2 run apriltag_ros_python apriltag_detection_server

ros2 run apriltag_ros_python apriltag_detection_client
```
