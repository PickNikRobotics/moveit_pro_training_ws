#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <apriltag_ros_msgs/srv/get_april_tag_detections.hpp>

namespace moveit_studio_training_behaviors
{

using GetDetectionsService = apriltag_ros_msgs::srv::GetAprilTagDetections;

/**
 * @brief 
 * 
 * @details
 * | Data Port Name | Port Type | Object Type                     |
 * | -------------- | --------- | --------------------------------|
 * | apriltag_id    | input     | int                             |
 * | file_prefix    | input     | sensor_msgs::msg::CameraInfo    |
 * | image          | input     | sensor_msgs::msg::Image         |
 * | detection_pose | output    | geometry_msgs::msg::PoseStamped |
 */
class GetAprilTagDetectionPose : public moveit_studio::behaviors::ServiceClientBehaviorBase<GetDetectionsService>
{
public:
  /**
   * @brief Constructor for the get_apriltag_detections behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  GetAprilTagDetectionPose(const std::string &name, const BT::NodeConfiguration &config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the get_apriltag_detections Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return get_apriltag_detections does not use expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief TODO 
   */
  fp::Result<std::string> getServiceName() override;

  /**
   * @brief TODO 
   */
  fp::Result<GetDetectionsService::Request> createRequest() override;

  /**
   * @brief TODO 
   */
  fp::Result<bool> processResponse(const GetDetectionsService::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return response_future_;
  }

  /** @brief Holds the result of calling the service asynchronously. */
  std::shared_future<fp::Result<bool>> response_future_;

};
}  // namespace moveit_studio_training_behaviors
