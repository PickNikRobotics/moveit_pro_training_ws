#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_training_behaviors/get_apriltag_detection_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
  constexpr auto kPortIDAprilTagId = "apriltag_id";
  constexpr auto kPortIDCameraInfo = "camera_info";
  constexpr auto kPortIDImage = "image";
  constexpr auto kPortIDDetectionPose = "detection_pose";

  constexpr auto kGetAprilTagDetectionsServiceName = "/detect_apriltags";
}  // namespace

namespace moveit_studio_training_behaviors
{
  GetAprilTagDetectionPose::GetAprilTagDetectionPose(const std::string &name, const BT::NodeConfiguration &config,
                                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources)
      : moveit_studio::behaviors::ServiceClientBehaviorBase<GetDetectionsService>(name, config, shared_resources)
  {
  }

BT::PortsList GetAprilTagDetectionPose::providedPorts()
{
  return {
    BT::InputPort<int>(kPortIDAprilTagId),
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo),
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDDetectionPose),
  };
}

fp::Result<std::string> GetAprilTagDetectionPose::getServiceName() {
  return kGetAprilTagDetectionsServiceName;
}

fp::Result<GetDetectionsService::Request> GetAprilTagDetectionPose::createRequest()
{
  const auto apriltag_id = getInput<std::string>(kPortIDAprilTagId);
  const auto camera_info = getInput<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo);
  const auto image = getInput<sensor_msgs::msg::Image>(kPortIDImage);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(apriltag_id, camera_info, image); error)
  {
    return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
  }

  GetDetectionsService::Request request;
  request.camera_info = camera_info.value();
  request.image = image.value();
  return request;
}

fp::Result<bool> GetAprilTagDetectionPose::processResponse(const GetDetectionsService::Response &response)
{
  // TODO: Filter by detection ID
  for (const auto& detection : response.detections)
  {
    std::cout << "Got detection with ID " << detection.id << std::endl;
  }

  return true;
}

}  // namespace moveit_studio_training_behaviors

template class moveit_studio::behaviors::ServiceClientBehaviorBase<moveit_studio_training_behaviors::GetDetectionsService>;
