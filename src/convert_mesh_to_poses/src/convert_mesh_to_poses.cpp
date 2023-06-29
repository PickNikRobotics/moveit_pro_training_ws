#include <convert_mesh_to_poses/convert_mesh_to_poses.hpp>

namespace convert_mesh_to_poses
{
ConvertMeshToPoses::ConvertMeshToPoses(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList ConvertMeshToPoses::providedPorts()
{
  // TODO(...)
  return BT::PortsList({});
}

BT::NodeStatus ConvertMeshToPoses::tick()
{
  // TODO(...)
  // Return SUCCESS once the work has been completed.

  std::cerr<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace convert_mesh_to_poses
