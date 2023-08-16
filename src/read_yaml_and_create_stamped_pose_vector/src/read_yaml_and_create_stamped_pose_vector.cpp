#include <read_yaml_and_create_stamped_pose_vector/read_yaml_and_create_stamped_pose_vector.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/parameter_tools.hpp>

#include <yaml-cpp/yaml.h>

#include <vector>

namespace
{
// Port names for input and output ports.
constexpr auto kPortIDYAMLPoses = "input_poses";
constexpr auto kPortIDPoseVector = "output_pose_vector";

constexpr auto kFrameId = "reference_frame";
}  // namespace

namespace read_yaml_and_create_stamped_pose_vector
{
ReadYamlAndCreateStampedPoseVector::ReadYamlAndCreateStampedPoseVector(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList ReadYamlAndCreateStampedPoseVector::providedPorts()
{
  return {
    BT::InputPort<YAML::Node>(kPortIDYAMLPoses),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPoseVector),
  };
}

BT::NodeStatus ReadYamlAndCreateStampedPoseVector::tick()
{

  const auto objective_parameters = getInput<YAML::Node>(kPortIDYAMLPoses);

  if (const auto error = moveit_studio::behaviors::maybe_error(objective_parameters); error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  const auto poses = moveit_studio::behaviors::parseParameter<YAML::Node>(objective_parameters.value(), name());
  if (fp::has_error(poses))
  {
    shared_resources_->logger->publishFailureMessage(
        name(),
        "Could not find behavior specific parameters in the configuration file: " + poses.error().what);
    return BT::NodeStatus::FAILURE;
  }

  const auto pose_yaml = poses.value();
  const auto num_poses = pose_yaml.size();  

  if (num_poses <= 0)
  {
    shared_resources_->logger->publishFailureMessage(name(), "There were no valid poses in the yaml file.");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped stamped_pose;
  std::vector<geometry_msgs::msg::PoseStamped> pose_vector;

  for (auto const& pose:pose_yaml){

    stamped_pose.header.frame_id = pose["reference_frame"].as<std::string>();
    stamped_pose.header.stamp = shared_resources_->node->now();

    stamped_pose.pose.position.x = pose["x"].as<double>();
    stamped_pose.pose.position.y = pose["y"].as<double>();
    stamped_pose.pose.position.z = pose["z"].as<double>();
    
    stamped_pose.pose.orientation.x = pose["qx"].as<double>();
    stamped_pose.pose.orientation.y = pose["qy"].as<double>();
    stamped_pose.pose.orientation.z = pose["qz"].as<double>();
    stamped_pose.pose.orientation.w = pose["qw"].as<double>();  

    pose_vector.push_back(stamped_pose);

  }

  setOutput(kPortIDPoseVector, pose_vector);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace read_yaml_and_create_stamped_pose_vector
