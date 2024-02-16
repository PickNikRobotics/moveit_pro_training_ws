# Creating Behaviors that Call External ROS Services

MoveIt Pro provides several convenient BehaviorTree.CPP nodes that help interact with ROS 2.
These include:

* `SharedResourcesNode<NodeType>`: Pass common resources in like a ROS node handle, TF buffer, etc.
* `AsyncBehaviorBase`: Allow ticking a Behavior instead of blocking while waiting for long-running processes.
* `GetMessageFromTopic` / `ServiceClientBehaviorBase` / `ActionClientBehaviorBase`: For common ROS 2 interoperability tasks.

You can find information about these nodes in the [API documentation](https://docs.picknik.ai/en/stable/doxygen/moveit_studio_behavior_interface/html/annotated.html).

## Implementing our AprilTag Detection Behavior

Since our AprilTag node is a ROS 2 service server, we can write a Behavior that calls it using a `ServiceClientBehaviorBase` node.
Packages that use this node type can't yet be created automatically, but you can use the structure from the [Custom Behaviors section](./04_custom_behaviors.md).

For this example, we're creating a package named `get_apriltag_detection_pose`.

The service is of type `apriltag_ros_msgs::srv::GetAprilTagDetections`, so we must ensure to do the following:

- Ensure the `package.xml` file depends on the following packages.
  ```xml
  <depend>apriltag_ros_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  ```
- Ensure the `CMakeLists.txt` file also builds the Behavior library by linking against these packages.
  ```cmake
  find_package(apriltag_ros_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  set(
    moveit_studio_behavior_interface
    pluginlib
    THIS_PACKAGE_INCLUDE_DEPENDS
    apriltag_ros_msgs
    geometry_msgs
    sensor_msgs
  )
  ```
- Include the service and message definition headers in the Behavior header file:
  ```cpp
  #include <apriltag_ros_msgs/srv/get_april_tag_detections.hpp>
  #include <geometry_msgs/msg/pose_stamped.hpp>
  #include <sensor_msgs/msg/camera_info.hpp>
  #include <sensor_msgs/msg/image.hpp>
  #include <std_msgs/msg/header.hpp>
  ```
- Define a class that inherits from `ServiceClientBehaviorBase` for our required service type.
  ```cpp
  namespace get_apriltag_detection_pose
  {

  using GetDetectionsService = apriltag_ros_msgs::srv::GetAprilTagDetections;

  class GetApriltagDetectionPose : public moveit_studio::behaviors::ServiceClientBehaviorBase<GetDetectionsService>
  {
  public:
  /** @brief Constructor for the get_apriltag_detections behavior. */
  GetApriltagDetectionPose(const std::string &name, const BT::NodeConfiguration &config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources);

  /** @brief Implementation of the required providedPorts() function for this Behavior. */
  static BT::PortsList providedPorts();

  private:
  /** @brief Returns the AprilTag detection service name. */
  tl::expected<std::string, std::string> getServiceName() override;

  /** 
  * @brief Packages the service request.
  * @details This request takes camera info and image messages from the blackboard input ports to this Behavior.
  */
  tl::expected<GetDetectionsService::Request, std::string> createRequest() override;

  /**
  * @brief Processes the service response.
  * @details Looks for the first detection instance that matches the specified ID, and if available sets its pose to the blackboard output port.
  */
  tl::expected<bool, std::string> processResponse(const GetDetectionsService::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return response_future_;
  }

  /** @brief Holds the result of calling the service asynchronously. */
  std::shared_future<tl::expected<bool, std::string>> response_future_;

  /** @brief The target AprilTag ID to look for. */
  int target_id_;

  /** @brief The image message header, used for publishing to TF tree. */
  std_msgs::msg::Header image_header_;
  };
  }  // namespace get_apriltag_detection_pose
  ```

This then involves implementing the override methods:

- Implement `providedPorts()`:
  ```cpp
  BT::PortsList GetApriltagDetectionPose::providedPorts()
  {
    return {
      BT::InputPort<int>("apriltag_id"),
      BT::InputPort<double>("apriltag_size"),
      BT::InputPort<sensor_msgs::msg::CameraInfo>("camera_info"),
      BT::InputPort<sensor_msgs::msg::Image>("image"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("detection_pose"),
    };
  }
  ```
- Implement `getServiceName()`:
  ```cpp
  tl::expected<std::string, std::string> GetApriltagDetectionPose::getServiceName() {
    return "/detect_apriltags";
  }
  ```
- Implement `createRequest()`:
  ```cpp
  tl::expected<GetDetectionsService::Request, std::string> GetApriltagDetectionPose::createRequest()
  {
    // Check that all required input data ports were set.
    const auto apriltag_id = getInput<int>("apriltag_id");
    const auto apriltag_size = getInput<double>("apriltag_size");
    const auto camera_info = getInput<sensor_msgs::msg::CameraInfo>("camera_info");
    const auto image = getInput<sensor_msgs::msg::Image>("image");
    if (const auto error = moveit_studio::behaviors::maybe_error(apriltag_id, apriltag_size, camera_info, image); error)
    {
      return tl::make_unexpected("Missing input port: " + error.value());
    }
    target_id_ = apriltag_id.value();
    image_header_ = image.value().header;

    // Create and return the service request.
    GetDetectionsService::Request request;
    request.apriltag_size = apriltag_size.value();
    request.camera_info = camera_info.value();
    request.image = image.value();
    return request;
  }
  ```
- Implement `processResponse()`:
  ```cpp
  tl::expected<bool, std::string> GetApriltagDetectionPose::processResponse(const GetDetectionsService::Response &response)
  {
    // Filter by detection ID. Simply get the first instance of a particular ID, if one is found.
    for (const auto& detection : response.detections)
    {
      if (detection.id == target_id_)
      {
        setOutput(kPortIDDetectionPose, detection.pose);

        // Publish detection to the TF tree.
        geometry_msgs::msg::TransformStamped tform;
        tform.header = image_header_;
        tform.child_frame_id = "apriltag_" + std::to_string(detection.id);
        tform.transform.translation.x = detection.pose.pose.position.x;
        tform.transform.translation.y = detection.pose.pose.position.y;
        tform.transform.translation.z = detection.pose.pose.position.z;
        tform.transform.rotation.w = detection.pose.pose.orientation.w;
        tform.transform.rotation.x = detection.pose.pose.orientation.x;
        tform.transform.rotation.y = detection.pose.pose.orientation.y;
        tform.transform.rotation.z = detection.pose.pose.orientation.z;
        shared_resources_->transform_broadcaster.sendTransform(tform);

        return true;
      }
    }

    // If no matching detections were found, this Behavior should fail.
    return tl::make_unexpected(
      std::string("Did not find any AprilTag detections with ID: ").append(std::to_string(target_id_)));
  }
  ```

Lastly, ensure this line is at the bottom of your `.cpp` file:

```cpp
template class moveit_studio::behaviors::ServiceClientBehaviorBase<get_apriltag_detection_pose::GetDetectionsService>;
```

This finished Behavior is available for reference at [get_apriltag_detection_pose.cpp](../src/solution_get_apriltag_detection_pose/src/get_apriltag_detection_pose.cpp).
