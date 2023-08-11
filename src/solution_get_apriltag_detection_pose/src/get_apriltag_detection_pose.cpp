#include <get_apriltag_detection_pose/get_apriltag_detection_pose.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
  constexpr auto kPortIDAprilTagId = "apriltag_id";
  constexpr auto kPortIDAprilTagSize = "apriltag_size";
  constexpr auto kPortIDCameraInfo = "camera_info";
  constexpr auto kPortIDImage = "image";
  constexpr auto kPortIDDetectionPose = "detection_pose";

  constexpr auto kGetAprilTagDetectionsServiceName = "/detect_apriltags";
}  // namespace

namespace get_apriltag_detection_pose
{
GetApriltagDetectionPose::GetApriltagDetectionPose(const std::string &name, const BT::NodeConfiguration &config,
                                                   const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources)
    : moveit_studio::behaviors::ServiceClientBehaviorBase<GetDetectionsService>(name, config, shared_resources)
{
}

BT::PortsList GetApriltagDetectionPose::providedPorts()
{
  return {
    BT::InputPort<int>(kPortIDAprilTagId),
    BT::InputPort<double>(kPortIDAprilTagSize),
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo),
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDDetectionPose),
  };
}

fp::Result<std::string> GetApriltagDetectionPose::getServiceName() {
  return kGetAprilTagDetectionsServiceName;
}

fp::Result<GetDetectionsService::Request> GetApriltagDetectionPose::createRequest()
{
  // Check that all required input data ports were set.
  const auto apriltag_id = getInput<int>(kPortIDAprilTagId);
  const auto apriltag_size = getInput<double>(kPortIDAprilTagSize);
  const auto camera_info = getInput<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo);
  const auto image = getInput<sensor_msgs::msg::Image>(kPortIDImage);
  if (const auto error = moveit_studio::behaviors::maybe_error(apriltag_id, apriltag_size, camera_info, image); error)
  {
    return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
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

fp::Result<bool> GetApriltagDetectionPose::processResponse(const GetDetectionsService::Response &response)
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
  return tl::make_unexpected(fp::Internal(
      std::string("Did not find any AprilTag detections with ID: ").append(std::to_string(target_id_))));
}

}  // namespace get_apriltag_detection_pose

template class moveit_studio::behaviors::ServiceClientBehaviorBase<get_apriltag_detection_pose::GetDetectionsService>;
