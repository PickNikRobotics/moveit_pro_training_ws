# Creating Behaviors that Call External ROS Services

MoveIt Studio provides several convenient BehaviorTree.CPP nodes that help interact with ROS 2.
These include:

* `SharedResourcesNode<NodeType>`: Pass common resources in like a ROS node handle, TF buffer, etc.
* `AsyncBehaviorBase`: Allow ticking a Behavior instead of blocking while waiting for long-running processes.
* `GetMessageFromTopic` / `ServiceClientBehaviorBase` / `ActionClientBehaviorBase`: For common ROS 2 interoperability tasks.

Since our AprilTag node is a ROS 2 service server, we can write a Behavior that calls it using a `ServiceClientBehaviorBase` node.
These packages can't be created automatically, but you can use the structure from the [Custom Behaviors section](./04_custom_behaviors.md).
For this example, we're using a different namespace named `moveit_studio_training_behaviors`, but you can use your own as needed.

The service is of type `apriltag_ros_msgs::srv::GetAprilTagDetections`, sp we must ensure to do the following:

- Ensure the `package.xml` and `CMakeLists.txt` files depend on the `apriltag_ros_msgs` package (and any other ROS packages needed).
- Include the service definition headers:
  ```cpp
  #include <apriltag_ros_msgs/srv/get_april_tag_detections.hpp>
  ```
- Define a class that inherits from `ServiceClientBehaviorBase` for our required service type.
  ```cpp
  namespace moveit_studio_training_behaviors
  {

  using GetDetectionsService = apriltag_ros_msgs::srv::GetAprilTagDetections;

  class GetAprilTagDetectionPose : public moveit_studio::behaviors::ServiceClientBehaviorBase<GetDetectionsService>
  {
  public:
  /** @brief Constructor for the get_apriltag_detections behavior. */
  GetAprilTagDetectionPose(const std::string &name, const BT::NodeConfiguration &config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources);

  /** @brief Implementation of the required providedPorts() function for this Behavior. */
  static BT::PortsList providedPorts();

  private:
  /** @brief Returns the AprilTag detection service name. */
  fp::Result<std::string> getServiceName() override;

  /** 
  * @brief Packages the service request.
  * @details This request takes camera info and image messages from the blackboard input ports to this Behavior.
  */
  fp::Result<GetDetectionsService::Request> createRequest() override;

  /**
  * @brief Processes the service response.
  * @details Looks for the first detection instance that matches the specified ID, and if available sets its pose to the blackboard output port.
  */
  fp::Result<bool> processResponse(const GetDetectionsService::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return response_future_;
  }

  /** @brief Holds the result of calling the service asynchronously. */
  std::shared_future<fp::Result<bool>> response_future_;

  /** @brief The target AprilTag ID to look for. */
  int target_id_;
  };
  }  // namespace moveit_studio_training_behaviors
  ```

This then involves implementing the override methods:

- Implement `providedPorts()`:
  ```cpp
  BT::PortsList GetAprilTagDetectionPose::providedPorts()
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
  fp::Result<std::string> GetAprilTagDetectionPose::getServiceName() {
    return "/detect_apriltags";
  }
  ```
- Implement `createRequest()`:
  ```cpp
  fp::Result<GetDetectionsService::Request> GetAprilTagDetectionPose::createRequest()
  {
    // Check that all required input data ports were set.
    const auto apriltag_id = getInput<int>("apriltag_id");
    const auto apriltag_size = getInput<double>("apriltag_size");
    const auto camera_info = getInput<sensor_msgs::msg::CameraInfo>("camera_info");
    const auto image = getInput<sensor_msgs::msg::Image>("image");
    if (const auto error = moveit_studio::behaviors::maybe_error(apriltag_id, apriltag_size, camera_info, image); error)
    {
      return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
    }
    target_id_ = apriltag_id.value();

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
  fp::Result<bool> GetAprilTagDetectionPose::processResponse(const GetDetectionsService::Response &response)
  {
    // Filter by detection ID. Simply get the first instance of a particular ID, if one is found.
    for (const auto& detection : response.detections)
    {
      if (detection.id == target_id_)
      {
        setOutput(kPortIDDetectionPose, detection.pose);
        return true;
      }
    }

    // If no matching detections were found, this Behavior should fail.
    return tl::make_unexpected(fp::Internal(
      std::string("Did not find any AprilTag detections with ID: ").append(std::to_string(target_id_))));
  }
  ```

Lastly, ensure this line is at the bottom of your `.cpp` file:

```cpp
template class moveit_studio::behaviors::ServiceClientBehaviorBase<moveit_studio_training_behaviors::GetDetectionsService>;
```
