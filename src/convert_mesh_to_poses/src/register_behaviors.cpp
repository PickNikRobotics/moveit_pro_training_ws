#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <convert_mesh_to_poses/convert_mesh_to_poses.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace convert_mesh_to_poses
{
class ConvertMeshToPosesBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ConvertMeshToPoses>(factory, "ConvertMeshToPoses", shared_resources);
    
  }
};
}  // namespace convert_mesh_to_poses

PLUGINLIB_EXPORT_CLASS(convert_mesh_to_poses::ConvertMeshToPosesBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
