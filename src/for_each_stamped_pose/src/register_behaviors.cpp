#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <for_each_stamped_pose/for_each_stamped_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace for_each_stamped_pose
{
class ForEachStampedPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>&) override
  {
    moveit_studio::behaviors::registerBehavior<moveit_studio::behaviors::ForEach<geometry_msgs::msg::PoseStamped>>(factory, "ForEachStampedPose");
    
  }
};
}  // namespace for_each_stamped_pose

PLUGINLIB_EXPORT_CLASS(for_each_stamped_pose::ForEachStampedPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
