# Working with MoveIt Studio Configuration Packages

### Adding a Custom ROS Node Service to a Launch Configuration

Consider if you needed another service to be available for a custom use-case.
For example, to detect April Tags, we would need a service like https://github.com/duckietown/lib-dt-apriltags to be available from within MoveIt Studio.  
The first step would be to adapt this into a ROS Service, which is already finished as an example here: [../src/apriltag_ros_python/](../src/apriltag_ros_python).

To automatically make this service start on launch, we will add it to the launch file:

- launch/sim/hardware_sim.launch.py

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

```python3
            camera_transforms_node,
            apriltag_server_node, # new code
        ]
    )
```

And that's it. Congratiulations, an April Tag detection service is now present in the Gazebo site config, enabling April Tag Behaviors to work.

### Adding a Scene Camera

We now consider the example of adding a Scene Camera to the site config.
This camera will be a Realsense d435 camera and serves as an overhead camera that can see the scene of the robot.

- config/cameras.yaml

We first add the configuration of the new Scene Camera to our Camera config file:

```python
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

- launch/sim/hardware_sim.launch.py
We need to get the images from Gazebo to ROS. Luckily, there exists a package for just this thing:
https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_image  
We will add a bridge for our new scene camera, one for each topic we want.
The remappings are simply to name the topics in a format that MoveIt Studio expects (specified above in cameras.yaml)
The topics we care about are the RGB image, the depth image, and the Camera Info.
First, the RGB and depth image topic bridges are added to the launch file under the `# Camera Topic Bridges #` comment (currently line 129 in the file):

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
            scene_image_rgb_gazebo_bridge,
            scene_image_depth_gazebo_bridge,
            scene_camera_info_gazebo_bridge,
```

- description/ur5e_gazebo.xacro

We first add an include to the hardware realsense_d435 macro (which is already found in the MoveIt Studio `picknik_accessories`) to the line after the realsense_d415 is added:

```xml
  <xacro:include filename="$(find picknik_accessories)/descriptions/sensors/realsense_d415.urdf.xacro" />
  <--! New include after this line -->
  <xacro:include filename="$(find picknik_accessories)/descriptions/sensors/realsense_d435.urdf.xacro" />
```

After the `external_camera_link` (currently on line 104):
```xml
  <xacro:realsense_d435 parent="external_camera_link" name="scene_camera" visible="false" simulate_depth="true">
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:realsense_d435>

  <!-- Additional Camera -->
  <xacro:if value="$(arg use_extra_camera)">
    <link name="extra_scene_camera_link" />
    <joint name="extra_scene_camera_joint" type="fixed">
      <parent link="world" />
      <child link="extra_scene_camera_link" />
      <origin xyz="-0.3 0.3 0.5" rpy="0.0 0.0 -3.14" />
    </joint>
    <xacro:realsense_d435 parent="extra_scene_camera_link" name="extra_camera" visible="false" simulate_depth="false">
      <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:realsense_d435>
  </xacro:if>

```

Congratulations! You should now have a scene camera displayed in the MoveIt Studio UI dropdown!
Feel free to change the location of this camera by modifying the `extra_scene_camera_joint`'s `origin` `xyz` for translation (in meters) or `rpy` for roll pitch yaw rotations (in radians).


