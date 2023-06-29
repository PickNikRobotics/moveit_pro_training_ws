#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <ik_batch/ik_batch.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace ik_batch
{
class IkBatchBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<IkBatch>(factory, "IkBatch", shared_resources);
    
  }
};
}  // namespace ik_batch

PLUGINLIB_EXPORT_CLASS(ik_batch::IkBatchBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
