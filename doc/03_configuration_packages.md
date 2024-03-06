# Working with MoveIt Pro Configuration Packages

### Overriding a Kinematics Solver Plugin

To change the Kinematics solver from the default (KDL IK) to another (such as TRAC-IK), add a `kinematics` override parameter to the `moveit_params` section in the `site_config.yaml` file so that it looks like this afterwards:

```python
# Override MoveIt parameters
moveit_params:
  servo:
    package: "ur_gazebo_config"
    path: "config/moveit/ur5e_gazebo_servo.yaml"
  kinematics:
    package: "ur_gazebo_config"
    path: "config/moveit/trac_ik_kinematics.yaml"
```

This updates the Gazebo configuration package to use the TRAC-IK inverse kinematics plugin instead of the default one defined in the parent configuration package.

### Adding a Custom ROS Node Service to a Launch Configuration

Consider if you needed another service to be available for a custom use-case.
For example, to detect AprilTags, we could use a library like https://github.com/duckietown/lib-dt-apriltags from within MoveIt Pro.  
The first step would be to adapt this into a ROS Service, which is already finished as an example here: [../src/apriltag_ros_python/](../src/apriltag_ros_python).

To automatically make this service start on launch, we will add it to the launch file `launch/sim/hardware_sim.launch.py`.

Add this at the end of the file just before the `return LaunchDescription(` portion:

```python

    # AprilTag detection node
    apriltag_server_node = Node(
        package="apriltag_ros_python",
        executable="apriltag_detection_server",
        output="both",
        parameters=[
            {
                "visualize": False,
                "apriltag_family": "tag36h11",
            }
        ],
    )
```

And finally, add it to the node list in `LaunchDescription`:

```python
    return LaunchDescription(
        [
            # original launch descriptions
            apriltag_server_node,  # new code
        ]
    )
```

And that's it. Congratulations: an AprilTag detection service is now present in the Gazebo configuration package, enabling AprilTag Behaviors to work.
If you want to test that things are working as intended you can go into a Terminal in the container and check the ROS 2 node and service lists:

```console
./moveit_pro shell
```

```console
$ ros2 node list | grep apriltag
  /apriltag_detection_server
$ ros2 service list | grep apriltag
  /apriltag_detection_server/describe_parameters
  /apriltag_detection_server/get_parameter_types
  /apriltag_detection_server/get_parameters
  /apriltag_detection_server/list_parameters
  /apriltag_detection_server/set_parameters
  /apriltag_detection_server/set_parameters_atomically
  /detect_apriltags
```

### Adding a Scene Camera

We now consider the example of adding a Scene Camera to the configuration.
This camera will be a RealSense d435 camera and serves as an overhead camera that can see the scene of the robot.

We first add the configuration of the new Scene Camera to our Camera configuration file at `config/wrist_camera.yaml`:

```yaml
- scene_camera:
    camera_name: "scene_camera"
    type: "sim"
    use: True
    # These values must match those specified in realsense_d435.urdf.xacro
    image_width: 640
    image_height: 480

    # information about the topics the camera publishes the raw image and info
    rgb_info: "/scene_camera/color/camera_info"
    rgb_image: "/scene_camera/color/image_raw"

    # By adding registered_rgb_depth_pair, this camera can be used for "Set Transform From Click"
    # Since this is simulation, assume registered image is simply the raw image.
    registered_rgb_depth_pair:
      depth_info: "/scene_camera/color/camera_info"
      depth_image: "/scene_camera/depth/image_rect_raw"
      registered_info: "/scene_camera/depth_registered/camera_info"
      registered_image: "/scene_camera/depth_registered/image_rect_raw"
```

We need to get the images from Gazebo to ROS. 
Luckily, there exists a package for just this thing: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_image  

We will add a bridge from this package for our new scene camera; one for each topic we want, in the file `launch/sim/hardware_sim.launch.py`.

The remappings are simply to name the topics in a format that MoveIt Pro expects (specified above in the Cameras yaml).
The topics we care about are the RGB image, the depth image, and the Camera Info.

First, the RGB and depth image topic bridges are added to the launch file under the `# Camera Topic Bridges #` comment:

```python
    # For the scene camera, enable RGB image topics only.
    scene_image_rgb_gazebo_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="scene_image_rgb_gazebo_bridge",
        arguments=[
            "/scene_camera/image",
        ],
        remappings=[
            ("/scene_camera/image", "/scene_camera/color/image_raw"),
        ],
        output="both",
    )
    scene_image_depth_gazebo_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="scene_image_depth_gazebo_bridge",
        arguments=[
            "/scene_camera/depth_image",
        ],
        remappings=[
            (
                "/scene_camera/depth_image",
                "/scene_camera/depth/image_rect_raw",
            ),
        ],
        output="both",
    )
```

Next, we add the bridge for the Camera Info below these: 

```python
    scene_camera_info_gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="scene_camera_info_gazebo_bridge",
        arguments=[
            "/scene_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        remappings=[
            ("/scene_camera/camera_info", "/scene_camera/color/camera_info"),
        ],
        output="both",
    )
```

And finally, we add these nodes to our `Launch Description` so they are actually launched:

```python
    return LaunchDescription
        [
            # original launch descriptions
            scene_image_rgb_gazebo_bridge,
            scene_image_depth_gazebo_bridge,
            scene_camera_info_gazebo_bridge,
        ]
```

We will add the hardware realsense_d435 to the simulation world by adding it to the simulation world xacro found at `description/ur5e_gazebo.xacro`.
To do this, we use the `realsense_d435` xacro macro (which is found in the `picknik_accessories` package and is already included in the simulation world xacro).

To add an instance of this macro, after the `external_camera_link` definition, add:

```xml
  <xacro:realsense_d435 parent="external_camera_link" name="scene_camera" visible="false" simulate_depth="true">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:realsense_d435>

  <!-- Additional Camera -->
  <link name="extra_scene_camera_link" />
  <joint name="extra_scene_camera_joint" type="fixed">
    <parent link="world" />
    <child link="extra_scene_camera_link" />
    <origin xyz="-0.3 0.3 0.5" rpy="0.0 0.0 -3.14" />
  </joint>
  <xacro:realsense_d435 parent="extra_scene_camera_link" name="extra_camera" visible="false" simulate_depth="false">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:realsense_d435>
```

Congratulations! You should now have a scene camera displayed in the UI dropdown!
Feel free to change the location of this camera by modifying the `extra_scene_camera_joint`'s `origin` `xyz` for translation (in meters) or `rpy` for roll pitch yaw rotations (in radians).


### Cheatsheet

If at any point you got lost or just want to skip to a working feature complete config you can update your `config.yaml` to point to finished configs:

- wrist_camera.yaml &rarr; wrist_and_scene_cameras.yaml
- ur5e_gazebo.xacro &rarr; ur5e_with_scene_camera_gazebo.xacro
- hardware_sim.launch.py &rarr; hardware_sim_cameras_apriltag.launch.py

Or if you want to skip this straight to the finished working `config.yaml`:

```bash
cd src/ur_gazebo_config
cp config/SOLUTION_cfg.yaml config/config.yaml
```
