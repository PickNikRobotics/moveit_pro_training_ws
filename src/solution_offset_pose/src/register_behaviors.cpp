#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <offset_pose/offset_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace offset_pose
{
  class OffsetPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
  {
  public:
    void registerBehaviors(BT::BehaviorTreeFactory &factory,
                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources) override
    {
      using namespace moveit_studio::behaviors;

      registerBehavior<OffsetPose>(factory, "OffsetPose");
    }
  };
} // namespace offset_pose

PLUGINLIB_EXPORT_CLASS(offset_pose::OffsetPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
