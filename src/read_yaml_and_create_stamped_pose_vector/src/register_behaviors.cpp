#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <read_yaml_and_create_stamped_pose_vector/read_yaml_and_create_stamped_pose_vector.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace read_yaml_and_create_stamped_pose_vector
{
class ReadYamlAndCreateStampedPoseVectorBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ReadYamlAndCreateStampedPoseVector>(factory, "ReadYamlAndCreateStampedPoseVector", shared_resources);
    
  }
};
}  // namespace read_yaml_and_create_stamped_pose_vector

PLUGINLIB_EXPORT_CLASS(read_yaml_and_create_stamped_pose_vector::ReadYamlAndCreateStampedPoseVectorBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
